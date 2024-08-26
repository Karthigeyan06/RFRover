#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

struct Data_Package {
  int xAxis;
  int yAxis;
  bool buttonPressed;
};

struct Sensor_Data {
  int distanceFront;
  int distanceLeft;
  int distanceRight;
  float temperature;
  float pressure;
  int gasValue;
};

Data_Package data;
Sensor_Data sensorData;

const int joyXPin = A0; 
const int joyYPin = A1; 
const int buttonPin = 2; 

bool lastButtonState = HIGH;
bool controlMode = true; 

void setup() {
  Serial.begin(9600);
  
  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP); 
 
  radio.begin();
  radio.openWritingPipe(0xF0F0F0F0E1LL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
  radio.startListening();
}

void loop() {

  data.xAxis = analogRead(joyXPin);
  data.yAxis = analogRead(joyYPin);

  
  bool buttonState = (digitalRead(buttonPin) == LOW);

  
  if (buttonState && lastButtonState == HIGH) {
    controlMode = !controlMode;
    delay(500); 
  }
  lastButtonState = buttonState;

  data.buttonPressed = controlMode;

 
  radio.stopListening();
  radio.write(&data, sizeof(Data_Package));
  radio.startListening();

  
  if (!controlMode && radio.available()) {
    radio.read(&sensorData, sizeof(Sensor_Data));
    displaySensorData();
  }

  delay(100);
}

void displaySensorData() {
  Serial.print("Distance Front: ");
  Serial.print(sensorData.distanceFront);
  Serial.print(" | Distance Left: ");
  Serial.print(sensorData.distanceLeft);
  Serial.print(" | Distance Right: ");
  Serial.print(sensorData.distanceRight);
  Serial.print(" | Temp: ");
  Serial.print(sensorData.temperature);
  Serial.print(" | Pressure: ");
  Serial.print(sensorData.pressure);
  Serial.print(" | Gas Value: ");
  Serial.println(sensorData.gasValue);
}
