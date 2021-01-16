/* **************************************************************************************
 *  ir_rf_sser_cmd
 *  Program : IR RFSwitch Receiver SoftSerial 8 + 5 CH 
 *  Program ID : IR_RF_SER_V1.01
 *  Author : Nagesh
 *  Date   : 07-02-2017
 *  Description :
 *              1.IR, RCSwitch reciver and Softserial(BT/Serial interface modules) with Temperature and Light Intensity monitoring. 
 *              2.8 Digital and 5 Analog(PWM) Channels.
 *              3.Configurable and processing from serial port connection.
 *              4.Serial Command functions to set/get/configure the pins and parameters.
 *              5.Can attach BT/LoRa/NFC/ESP/WiFi to softserial and 2 way communication can be done.
 *              6.For Arduino Uno,Nano,Pro Mini.
 * Update : 27-02-2017:            
 *         Code aligned for Pro Micro 328 (3.3 V,8 Mhz).
 *         RF,IR,Serial Ports for Controling Digital/Analog Pins.
 *         RF Switch Lib code send format : <devid-3 digit><Cmd-1 digit><Pin_Index - 1 Digit>
 *          Device ID= 3 digit Unique identifer fr the device
 *          CMD  = 1 Digit code  (0 : Digital Low,1: Digital High,2;Analog Decrement,3:Analog Increment, 5: Toggel Digital )
 *          Pin Index = Analog(1-5) / Digital(1-8) Pin index, Analog and Digital Pin index starts with 1
 *             Eg: code : 10201  => Device Id=102 ,Digital Pin Index=1, cmd=0(Digital LOW) => set Digital Pin 1 to LoW
 *                 code : 10213  => Device Id=102 ,Digital Pin Index=3, cmd=1(Digital HIGH) => set Digital Pin 3 to HIGH
 *                 code : 10231  => Device Id=102 ,Analog Pin Index=1, cmd=3(Analog Increment setp) => Increment Analog Pin 1 with predefined step (default step=25)
 *                 
 ****************************************************************************************/
/*CMD Functions:  
  <X><Y><Z>  :- X:{S:Set,G:Get,T:Toggel}, Y:{D:Digital,A:Analog;S:Status,T:Temperature,E:Eprom}, 
  *       Z:{A: All,I:Incremnet,D:Decrement,R: Resistance,S:Step}
  1. GD <Pin> <0> : getpinD(): Get the digital pin status/value (0/1)
  2. GA <Pin> <0> : getpinA(): Get the Analog pin status/value (0-1023)
  3. SD <Pin> <val> : setpinD(): Set the digital pin status/value  (0/1)    
  4. SA <Pin> <val> : setpinA(): Set the Analog pin value  (0-255)  -- Pins 3,5,6,9,10,11
  5. TD <Pin> <0> : toggleD(): Toggle the Digital pin.
  6. GT <0> <0> : getTempT(): Get the temperature in Centigrade read from Thermistor. - PIN A6
  7. GL <0> <0> : getIntensity(): Get the light Intensity. -- PIN A7
  8. GS <0> <0> : getStatus(): Get the status of all the pins.
  9. GR <0> <0> : Get Thermistor Resistance set in K Ohms 
 10. SR <0> <0> : Set Thermistior Resistance in K Ohms
 11. GV <0> <0> : Get PIN Analog/Digital values from configuration table/array.
 12. GT <0> <0> : Get Temperature (Thermistor) at PIN A6.
 13. GL <0> <0> : Get Light Intensity (LDR) at PIN A7.
 14. SD 0 <val> : setAllpinD(): Set All the digital pin status/value  (0/1)   
 15. SA 0 <val> : setAllpinA(): Set All the Analog pin value  (0-255)  -- Pins 3,5,6,9,10,11
 16. ID <0|1> [<dname>] : Get/set Device Id.
 17. AI <pin> <up/dn> : Analog Increment/Decrement PWM pin with a predefined step IncAnalog.
 18. RESET      : Reset/Restart the unit/processer.
 19. FACTRESET : Factory Reset - Copies default initial values to persistant and uses these config values.
 20. SB <01> [<baud>] : set/get Baud rate for serial communication  
process_request() : Process the request/command from client/serial input.
  
IR Processing :
 Pin A0 to A5,12,13 as Output for n- Channel Relay.  (Nano/ mini)
 Pin 5,6,9,10,11 as analog (up/dn) out put  

Serial Processing:
Pin 7 & 8 : Rx/Tx for Serial port processing.

Monitoring:
 Pin A6 : Temperature
 Pin A7 : Light Intensity

EPROM Bytes:
 1. 0  - 64: {[0-4 bytes : H/W Ver. M.m.n.c],[5-8 bytes : S/W Ver. S.s.ss.x],[9-12 Byte: Model-Type,Cate,mode],[13-16 Bytes:upd_date dd mm YY yy],[17-20 Byte: X.ROOM.LOC.ID] } - Meta data,Filled once at burining code
 2. 65 -128:  {[65-88 bytes: PIN Status/Val],[89-112 Byte : 16-Digital(89-104),8-Analog Pins(105-112)] ,[113-128 Bytes: IR pin,Ther,LDR,(BTx,BRx),(WTx,WRx),(STx,SRx)]}  --- Config/global values.
 3. 129 -192: {[IR Codes -Digital pins 8+2, 10 x 4 Bytes=40 Bytes]}
 4. 193 -256: {[IR Codes 12 x 4 bytes=48 bytes]}
 5. 257 -320: {[IR codes, Analog 5 + 1 UP/Dn]}
 6. 321 -384 : {[IR Codes secondary 12 x 2 bytes]}
 7. 385 -448: {[Global vars:Ther resis,]} 
 * 
*/

#include <EEPROM.h>
#include <IRremote.h>
#include <SoftwareSerial.h>
#include <VirtualWire.h>
#include <math.h>
#include "RCSwitch.h"
#include <avr/wdt.h>

//const int rf_tx_en_pin = 5;
 int DID=203;     // 1xx : Testing units, 2xx: Home Units. 

#define ST_ANALOG 5
#define ST_DIGITAL 8
#define MIS_PINS 9  // Pins - IR,Ther,LDR,BT(rx/tx),Wifi(rx/tx),RF(tx/rx)

int MAX_ANALOG=225;

//int STORE_EEPROM=65;   // eprom 65->88 : Pin Status values
int STORE_DIGITAL=512;   // eprom 512->534 : Pin value digtal
int STORE_ANALOG=535;   // eprom 535->567 : Pin  values analog
int EPR_ST_D_PIN=89;   // eprom 89-97 : 8 pins (12,13,14,15,16,17,18,19) , 
int EPR_ST_A_PIN=105;  // eprom 105-109 : 5 pins (5,6,9,10,11)
int EPR_ST_MIS_PIN=113;// eprom 113-128  [1.IR pin,2.Ther,3.LM35,(4.BTx,5.BRx),(6.WTx,7.WRx),(8.RFTx,9.RFRx)]

int EPR_IR_D_CODE=129;   // eprom IR Key codes 4 bytes per code, for each digital pin
int EPR_IR_A_CODE=257;   // eprom IR Key Codes 4 bytes per code. for Up/Down Keys of each Analog pin
int EPR_IR_D_CODE2=321;   // eprom IR Key Codes 4 bytes per code. for each digital pin --- secondary IR code2.
int EPR_ST_VAR_GLOBAL=385;  // eprom Global values , (ana_step 1-8 nos,9.Therm Resi.in K ohm,)
int EPR_ST_VAR_ID=449;  // 449-565 eprom Global values Device Id, (Name 1-8 nos,)
int EPR_FACT_RESET=466; //  Factory Reset at next reboot = 0: No Reset,1: Reset to defaults on reboot 
int EPR_BAUD_SET=475; //  Baud rate for Soft serial communication 

int st_Analog[ST_ANALOG]={5,6,9,10,11};
byte va_Analog[ST_ANALOG]={0,0,0,0,0};
byte F_RESET=0;
int st_Digital[ST_DIGITAL]={14,15,16,17,18,19,12,13};
bool va_Digital[ST_DIGITAL]={0,0,0,0,0,0,0,0};

long THE_RES=10000;  //10K default
const int incAnalog=25;

// Variables for IR processing
int IR_IN = 2; // NANO
//int IR_IN = 12;   //PRO MINI 8 Mhz
int THE_IN=20; // A6
int LDR_IN=21; //A7
// PIN for RX/TX for BT/Wifi/Serial IN/OUT devices.
 int RC_INT=1; //RF -RCSwitch INT1 pin => PIN 3.
 int rf_tx_pin = 4;
 int rf_rx_pin = 3; // for TX of RCswitch..
int RX=7;
int TX=8;

long S_BAUD=9600;

// IR Codes                    
long RCODE[10]={0xFDB04F,0xFD00FF,0xFD807F,0xFD40BF,0xFD20DF,0xFDA05F,0xFD609F,0xFD10EF,0xFD906F,0xFD50AF};  
//long RCODE[2][10]={{0x1FE7887,0xFFC03F,0xFF20DF7,0xFFA05F,0xFF609F,0xFFE01F,0x1FE708F,0x1FE00FF,0x1FEF0FF,0x1FE48B7},
//                    ,{0x1FE7887,0xFFC03F,0xFF20DF7,0xFFA05F,0xFF609F,0xFFE01F,0x1FE708F,0x1FE00FF,0x1FEF0FF,0x1FE48B7}};
long ACODE[2][6]={{0xFDA857,0xFD8877,0xFD28D7,0x1FEC03F,0x1FEC03F,0x1FEC03F},       //up
                   {0x1FE40BF,0xFD9867,0xFD6897,0x1FEC03F,0x1FEC03F,0x1FEC03F}};       //down

long RCODE2[10]={0xD,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0xC};  
long ircode;

int rval=0,devstate=0,irstate=0,rfstate=0,ircount=0; 
uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

RCSwitch mySwitch = RCSwitch();
IRrecv irrecv(IR_IN);
decode_results res;
SoftwareSerial Sserial(RX,TX);  // PIN:= RX=7,TX=8

char sreq[30];
String resp;

int thermistor;

char *req;

//void(* resetFunc) (void) = 0;

void setup() 
{
  F_RESET=EEPROM.read(EPR_FACT_RESET);
   Serial.begin(9600);
  if(F_RESET==0)init_eprom();
  else init_config(); 
  init_pins(); 
  setoldstate(); 
  irrecv.enableIRIn();  // enable IR Recption  
  vw_set_tx_pin(rf_tx_pin);
//  vw_set_rx_pin(rf_rx_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);       // Bits per sec
// //  vw_rx_start();  
  mySwitch.enableReceive(RC_INT);  // Receiver on inerrupt 1 => that is pin #3
  Sserial.begin(S_BAUD);
Serial.print("My ID=");Serial.println(DID);
Serial.println("PIN : IR=2,RF=(3,4)-RCSwitch,Serial=(7,8)-SoftSerial,LDR=21,THERM=20 and Default Serial=(0,1)");
Serial.println("Control PIN : Digital(14,15,16,17,18,19,12,13), Analog(5,6,9,10,11), With Serial Commands ...");
}

void loop()
{
   char sreq[30];   
 int i=0;
 sreq[i]='\0';
 devstate=0,irstate=-1,rfstate=-1;
 resp=handleIR();
if(Serial.available()>0){ 
 while(Serial.available()>0  && i<30){
    sreq[i]=Serial.read();      
    delay(5);
    i++;
  }
if(i>0)sreq[i]='\0';
 // Serial.print(" In Serial=");
 //Serial.println(sreq); 
 resp=process_request(sreq); 
  //Serial.println(resp);  
  } 
 else if (mySwitch.available()) 
  {  
   int value = mySwitch.getReceivedValue();
  
 //Serial.print(value);
   if(rval!=value)
   {rval=value;
    //Serial.print("processing....");
    if(DID==rval/100)resp=processSwitch(rval);
    }   
    mySwitch.resetAvailable();
  }
else if(Sserial.available()>0){ 
 while(Sserial.available()>0  && i<30){
    sreq[i]=Sserial.read();      
    delay(5);
    i++;
  }
if(i>0)sreq[i]='\0';
 // Sserial.println(sreq); 
  
 resp=process_request(sreq); 
  //Sserial.println(resp);  
  } 
  
  if(devstate==1){ savestate(); //Serial.println("Save State...");
  //resp=process_request("GV 0 0"); 
  //Serial.println(resp);  
      }
   if(resp!=""){Serial.println(resp);Sserial.println(resp);} 
}

//IR : Handle IR Request
String handleIR(){
  String resp="";
   if (irrecv.decode(&res)){
    // Handling Garbage values - ignoring 
    //Serial.println("in IR handle");
    //dump(&res);
    //Serial.println(res.value,HEX);
    
     if(res.decode_type != UNKNOWN){    
      resp=processIRcode(res.value);
     }   
    irrecv.resume();
     if((irstate!=0 && devstate==0) ){ // Forward the cmd ....          
      rfsend(res.value);
      resp="FW:"+res.value;
    }
    if(res.value==ircode ||  devstate==0)ircount++;
    else ircount=1;
    if(devstate!=0)ircount=1;
    irstate++;
    
    //Serial.println(ircount);
    if(ircount>20){
      irstate=0;
      Serial.println("Rebooting...!");Sserial.println("Rebooting...!");
      software_Reboot();       
      }   // reset if garbage values are more in sequence.
     ircode=res.value;
    }  
    return resp;  
}

// Process the IR code 
 String processIRcode(long value)
 { int res=99;
   int arow=99;
   int acol=99;
   int st=99;
   String resp="";
   char cmd[3]="SD";
   irstate++;
 
   for (int i=0;i<10 && res>50 ;i++){
     if(value==RCODE[i] ){res=i;st=0; cmd[0]='T'; break;}
     else if(value==RCODE2[i] ){res=i;st=1; break;}
     else if(value==RCODE2[i] + 0x800){res=i;st=0; break;}
      
    }
  // if digital switch found process it.
  
  if(res<50)
  {
   resp=processcmd(cmd,res,st);
   irstate=0;
   }
   else   // if ananlog up/dn 
    { for (int i=0;i<2 && arow==99 && acol==99;i++){
        for(int j=0;j<6 && arow==99 && acol==99;j++){           
          if(value==ACODE[i][j])
           {arow=i;acol=j;}
          }
      } 
   // Analog
   if (acol!=99){     
      if(arow==0) resp=processcmd("AI",acol,1);
      else  resp=processcmd("AD",acol,-1);   
      irstate=0;  
    }
    } 
 return resp;    
   }

//Process RFSwitch
String  processSwitch(int rval)
{
  int val=0;
  int code=rval % 100;
  int pin=code % 10;
  int cmd=code/10;
  String resp="N/A";
 rfstate++;
// Serial.print("SW:");Serial.println(rval);
 if(cmd==2 || cmd==3)
  { if(cmd==2)resp=processcmd("AD",pin,-1); //val=-1;
    else resp=processcmd("AI",pin,1);  //val=1; 
     rfstate=0;
  //anaIncDec(pin,val);   
 }
//Serial.println("SetD:");Serial.print(pin);Serial.print(", Val:");Serial.print(val);
  else if(cmd==0 || cmd==1)
  { if(cmd==0)val=0;
    else val=1; 
     rfstate=0;
 // setpinD(pin,val);
 resp=processcmd("SD",pin,val); 
 }
 else if(cmd==5)
  { 
 resp=processcmd("TD",pin,1); 
 }
 return resp;
}

//process request  - process the request and send/ack responce ---------------------------
// Request string : <Fn_name> [<PIN>] [<VAL>]
String process_request(char* req1){
 String res="";
// String treq = String(req);
 char cmd[5];
 //char *sparam1,*sparam2,*sval;
 int param1,val;
long param2;
 char c1,c2;
// Serial.println(req1); 
 int i=sscanf(req1,"%s %d %ld",&cmd,&param1,&param2); 
 // Serial.println(cmd);Serial.println(param1);Serial.println(param2);
  res=processcmd(cmd,param1,param2);  
 return res;
}

String processcmd(char cmd[5],int param1,int param2)
{
   String res="";
   int val;
    float tval1;
   // char ucmd[5]=toupper(cmd);
 //   Serial.println("in Cmd: ");Serial.print(cmd);Serial.print("- ");Serial.print(param1);Serial.print(" - ");Serial.println(param2);
  if(strcmp(cmd,"SD")==0 ){
     val=setpinD(param1,param2);
     res=res+ "P"+param1+":"+val;
  }
  else if (strcmp(cmd,"GD")==0){
     val=getpinD(param1);
    res=res+ "P"+param1+":"+val;
  }
 else if (strcmp(cmd,"SA")==0){
     val=setpinA(param1,param2);
     res=res+ "P"+param1+":"+val;
  }
 else if (strcmp(cmd,"GA")==0){
     val=getpinA(param1);
     res=res+ "P"+param1+":"+val;
  }
 else if (strcmp(cmd,"GS")==0){
    res=res+ getStatus();    
  }
   else if (strcmp(cmd,"GV")==0){
    res=res+ getValues();    
  }
 else if (strcmp(cmd,"TD")==0){
     val=toggleD(param1);
     res=res+ "P"+param1+":"+val;
  }
  else if (strcmp(cmd,"GT")==0){
     tval1=getTempT();
     res=res+ "T:"+tval1+" C";
   //res=res+ "Temp:"+dtoStr(tval1,5,2,10)+" C";
    //res="OK";
  }
  else if (strcmp(cmd,"GL")==0){
     tval1=getLDR();
     res=res+ "L:"+tval1+" ";
    //res="OK";
  }
  else if (strcmp(cmd,"SR")==0){
    val=setTher(param1);
    res=res+ "R:"+val +" K Ohm";  
  }
  else if (strcmp(cmd,"GR")==0){
    res=res+ "R:"+thermistor/1000 +" K Ohm";
  }

  else if (strcmp(cmd,"AR")==0){
     res=res+ "A"+param1+":"+analogRead(param1); 
  }
   else if (strcmp(cmd,"DR")==0){
     res=res+ "D"+param1+":"+digitalRead(param1); 
  }
  else if (strcmp(cmd,"AI")==0){
    val=anaIncDec(param1,1);
     res=res+ "A"+param1+":"+val; 
  }
   else if (strcmp(cmd,"AD")==0){
      val=anaIncDec(param1,-1);
     res=res+ "A"+param1+":"+val; 
  }
  else if (strcmp(cmd,"ID")==0)
  {
     if(param1==0){DID=getID();res="ID:"+DID;}
     if(param1==1){setID(param2);res="ID:"+DID;}  
  
  }
  else if (strcmp(cmd,"BD")==0)
  {
     if(param1==0){S_BAUD=getBaud();res="BD:"+S_BAUD;}
     if(param1==1){setBaud(param2);res="BD:"+S_BAUD;}  
  
  }
  else if (strcmp(cmd,"RESET")==0){
      Serial.println("Rebooting/Resetting.....");
      delay(500);
      software_Reboot();       
  }
  else if (strcmp(cmd,"FACTRESET")==0){
      Serial.println("Factory Reset.....");
    EEPROM.write(EPR_FACT_RESET,0);
    delay(500);
      software_Reboot();       
  }
  else  res="N/A";
 // Serial.print(res);
  return res;
}

//Analog Incremetnt
byte anaIncDec(int pin,int inc)
{ byte val=0;
  int ival=0;
// Serial.print("AI-");Serial.print(pin);Serial.print("-"); Serial.print(inc);Serial.print("-");  
   if (pin>0 && pin<6){
    //pinMode(st_Analog[pin-1],INPUT);
    ival=va_Analog[pin-1];
    ival=ival+ inc*incAnalog;
      if(ival>=250)val=255;
      else if(ival<incAnalog)val=0;
      else val=(byte)ival;
     if(val>=0 && val<=255){va_Analog[pin-1]=val;analogWrite(st_Analog[pin-1],val); devstate=1;}
//Serial.print(st_Analog[pin-1]);Serial.print("-"); 
   }
  else if(pin==0)
  {
    for (int i=0;i<ST_ANALOG;i++){va_Analog[i]=0; analogWrite(st_Analog[i],0);}
     devstate=1;
    val=0;
    }
      // Serial.print(ival);Serial.print("--");Serial.print(val);
 return val;
  }
  
//getTempT()
float getTempT(){
  float temp=10,aread,R;
 aread=analogRead(THE_IN);
 R = (1024/aread-1) * thermistor;
temp=log(R);
temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
  temp = temp - 273.15;  // Convert Kelvin to Celsius                      

return temp;
}
//getLDR()
float getLDR(){
float temp=35;
temp=analogRead(LDR_IN);

return temp/4;
}

//setTher(param1)   == param1 in K ohms
int setTher(int param1){
long temp=param1*1000;  //convert K ohms to ohms.
thermistor=temp;

return param1;
}

// setpinD(int pin) :set digital pin
int setpinD(int pin,int val) {
   bool bval;
   if (val!=0)bval=1;
   else bval=0; 
   if (pin==0 || pin==9){
    for(int i=0;i<ST_DIGITAL;i++){
      // pinMode(st_Digital[i],OUTPUT);
      va_Digital[i]=bval;
        digitalWrite(st_Digital[i],bval);
      }
      devstate=1;
    }
   else if(pin>0 && pin<9){
       devstate=1;        
        va_Digital[pin-1]=bval;
        digitalWrite(st_Digital[pin-1],bval);
      }

 return val;
}

// setpinA(int pin,inT val) :set Analog val , val=0->255
int setpinA(int pin,byte val) {
 if (val>=255)val=255;
 if (val<=0) val=0; 
 if (pin==0){
      for(int i=0;i<=ST_ANALOG;i++){  va_Analog[i]=val; analogWrite(st_Analog[i],val);}
      devstate=1;
    }
   else if (pin>0 && pin<6) {
        if(val>=0 && val<=255){  va_Analog[pin-1]=val; analogWrite(st_Analog[pin-1],val);}
         devstate=1;
      }
 return val;
}

// getpinD(int pinl) :get digital value
int getpinD(int pin){
  int val;
  digitalRead(st_Digital[pin-1]);
  return val;
}

// getpinA(int pinl) :get Analog value
int getpinA(int pin) {
 int val;
 analogRead(st_Analog[pin-1]);
 return val/4;
}

//toggleD(pin) :toggel Pin 
int toggleD(int pin) {
   bool val;
   if (pin==0 || pin==9){
    if (pin==0)val=0;
    else val=1;
    for(int i=0;i<ST_DIGITAL;i++){ 
      va_Digital[i]=val;     
        digitalWrite(st_Digital[i], val);
      }
       devstate=1;
   }
    else {
      val=!digitalRead(st_Digital[pin-1]);
      devstate=1;
      va_Digital[pin-1]=val;
    digitalWrite(st_Digital[pin-1],val);
    }

 return val;
}

//getStatus(): get the stauts of pins analog/digital in a string
String getStatus() {
  String pinstat="A=";
 for(int i=0;i<ST_ANALOG;i++) {  
  // pinMode(st_Analog[i],INPUT);       
   pinstat=pinstat+st_Analog[i]+":"+analogRead(st_Analog[i])+",";
 } 
  pinstat=pinstat+"D=";
  for(int i=0;i<ST_DIGITAL;i++) {         
   pinstat=pinstat+st_Digital[i]+":"+digitalRead(st_Digital[i])+",";
 }   
   return pinstat;
}

//getValues(): get the stauts of pins analog/digital in a string
String getValues() {  
String pinstat="ID=";
   pinstat=pinstat+DID+",A=";
 for(int i=0;i<ST_ANALOG;i++) {  
  // pinMode(st_Analog[i],INPUT);       
   pinstat=pinstat+st_Analog[i]+":"+va_Analog[i]+",";
 } 
  pinstat=pinstat+"D=";
  for(int i=0;i<ST_DIGITAL;i++) {         
   pinstat=pinstat+st_Digital[i]+":"+va_Digital[i]+",";
 }   
 
   return pinstat+"BD:"+S_BAUD;
}
  
void setoldstate(){   // read data from eeprom - old state of output pins.
   //Serial.print("Digit:");
   for( int i=0;i<8;i++)digitalWrite(st_Digital[i],va_Digital[i]);
   //  Serial.print("Analog:");
   for( int i=0;i<5;i++)analogWrite(st_Analog[i],va_Analog[i]);
}
 
void savestate(){   // save state of output pins.
  // for digital          
      EEPROM.put(STORE_DIGITAL,va_Digital);
    //for analog  st_Analog
      EEPROM.put(STORE_ANALOG,va_Analog);
}

void init_pins(){
  for(int i=0;i<ST_DIGITAL;i++)pinMode(st_Digital[i],OUTPUT);
  for(int i=0;i<ST_ANALOG;i++)pinMode(st_Analog[i],OUTPUT);
  // for digital          
  EEPROM.get(STORE_DIGITAL,va_Digital);
  //for analog  st_Analog
  EEPROM.get(STORE_ANALOG,va_Analog);
}

void rfsendtxt(char *msg,int len){
  vw_send((uint8_t *)msg, len);
  vw_wait_tx(); // Wait until the whole message is gone
  }

void rfsend(long ir_val){
  //char *cval
  vw_send((uint8_t *)ir_val, 5);
  vw_wait_tx(); // Wait until the whole message is gone
  }

/* ***********************************************************
 * Initilize the configuration values from the eprom storage
 *  1. Get pin nos analog/digital pins array
 *  2. Get
 * ***********************************************************/
void init_config(){ 
  // 1. Pin Nos from Eprom
  for(int i=0;i<ST_DIGITAL;i++)st_Digital[i]=EEPROM.read(EPR_ST_D_PIN+i);
  for(int i=0;i<ST_ANALOG;i++){
    st_Analog[i]=EEPROM.read(EPR_ST_A_PIN+i); //PIN    
  }
  
  // 2. Pin Values mis IR/BT/RF/
  //[1.IR pin,2.Ther,3.LM35,(4.BTx,5.BRx),(6.WTx,7.WRx),(8.RFTx,9.RFRx)]
  IR_IN=EEPROM.read(EPR_ST_MIS_PIN+0);
  THE_IN=EEPROM.read(EPR_ST_MIS_PIN+1);
  LDR_IN=EEPROM.read(EPR_ST_MIS_PIN+2);
 rf_rx_pin=EEPROM.read(EPR_ST_MIS_PIN+3);
  rf_tx_pin=EEPROM.read(EPR_ST_MIS_PIN+4);
  DID=EEPROM.read(EPR_ST_VAR_ID);
  S_BAUD=EEPROM.read(EPR_BAUD_SET);
  // 3. constants/global values (ther resis in K ohm,...)
  // eprom Global values , (ana_step 1-8 nos,9.Therm Resi.in K ohm,)
THE_RES=EEPROM.read(EPR_ST_VAR_GLOBAL+8)*1000;
  // IR codes 
      EEPROM.get(EPR_IR_D_CODE,RCODE);
      EEPROM.get(EPR_IR_A_CODE,ACODE);
      EEPROM.get(EPR_IR_D_CODE2,RCODE2);     
  }
  
/* ***********************************************************
 * Initilize the eprom values once 
 *  1. Analog/digital pins array
 *  2. 
 * ***********************************************************/ 
void init_eprom(){   // This to be run once at install, or for factory reset.
  // 1. Pin Nos from Eprom  EEPROM.write(EPR_ST_D_PIN+i,st_Digital[i]);
  for(int i=0;i<ST_DIGITAL;i++)EEPROM.write(EPR_ST_D_PIN+i,st_Digital[i]);
  for(int i=0;i<ST_ANALOG;i++){
    EEPROM.write(EPR_ST_A_PIN+i,st_Analog[i]);  //PINs   
  }
  
  // 2. Pin Values mis IR/BT/RF/
  //[1.IR pin,2.Ther,3.LM35,(4.BTx,5.BRx),(6.WTx,7.WRx),(8.RFTx,9.RFRx)]
  EEPROM.write(EPR_ST_MIS_PIN+0,IR_IN);
  EEPROM.write(EPR_ST_MIS_PIN+1,THE_IN);
  EEPROM.write(EPR_ST_MIS_PIN+2,LDR_IN);
  EEPROM.write(EPR_ST_MIS_PIN+3,rf_rx_pin);
  EEPROM.write(EPR_ST_MIS_PIN+4,rf_tx_pin);
    
  // 3. constants/global values (ther resis in K ohm,...)
  // eprom Global values , (ana_step 1-8 nos,9.Therm Resi.in K ohm,)  
  EEPROM.write(EPR_ST_VAR_GLOBAL+8,(int)THE_RES/1000);
  EEPROM.put(EPR_IR_D_CODE,RCODE);
  EEPROM.put(EPR_IR_A_CODE,ACODE);
  EEPROM.put(EPR_IR_D_CODE2,RCODE2);
  EEPROM.write(EPR_ST_VAR_ID,DID);
  EEPROM.write(EPR_FACT_RESET,255);
  EEPROM.write(EPR_BAUD_SET,S_BAUD);
  // Values for digital          
      EEPROM.put(STORE_DIGITAL,va_Digital);
  //Values for analog  st_Analog
      EEPROM.put(STORE_ANALOG,va_Analog);
  }

long setBaud(long baud)
{  EEPROM.write(EPR_BAUD_SET,baud);
S_BAUD=baud;
return baud;
 }
 
long getBaud()
{long baud;
  baud= EEPROM.read(EPR_BAUD_SET);
  return baud;
 }
 
int setID(int ID)
{  EEPROM.write(EPR_ST_VAR_ID,ID);
DID=ID;
return ID;
 }
 
int getID()
{int ID;
  ID= EEPROM.read(EPR_ST_VAR_ID);
  return ID;
 }
 
//Reboot 
void software_Reboot(){
  wdt_enable(WDTO_15MS);
  while(1){}
}
