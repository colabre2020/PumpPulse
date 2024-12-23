#include <DHT.h>  // Include necessary libraries
#define DHTPIN 2  // Pin where DHT sensor is connected
#define DHTTYPE DHT11  // DHT11 sensor
#define pumpPin 8  // Pin to control water pump
#define moisturePin A0  // Pin for soil moisture sensor

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(pumpPin, OUTPUT);  // Set water pump pin as output
  pinMode(moisturePin, INPUT);  // Set moisture sensor pin as input
  dht.begin();  // Initialize DHT sensor
}

void loop() {
  int moistureLevel = analogRead(moisturePin);  // Read moisture level
  float temp = dht.readTemperature();  // Read temperature
  float humidity = dht.readHumidity();  // Read humidity
  
  Serial.print("Moisture Level: ");  // Debug output
  Serial.println(moistureLevel);
  Serial.print("Temperature: ");
  Serial.println(temp);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  if (moistureLevel < 500) {  // Threshold for moisture (can be adjusted)
    digitalWrite(pumpPin, HIGH);  // Turn on water pump
    Serial.println("Water pump ON");
  } else {
    digitalWrite(pumpPin, LOW);  // Turn off water pump
    Serial.println("Water pump OFF");
  }
  
  delay(5000);  // Wait for 5 seconds before reading again
}
