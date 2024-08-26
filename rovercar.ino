#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>

RF24 radio(9, 10); 
Servo myServo;
Adafruit_BMP085_Unified bmp(10085);

const int in1 = 5;
const int in2 = 6;
const int in3 = 7;
const int in4 = 8;
const int ena = 3;
const int enb = 4;
const int triggerPin = A1;
const int echoPin = A2;
const int mq2Pin = A0;

struct Data_Package {
  int xAxis;
  int yAxis;
  bool buttonPressed;
};

Data_Package data;
struct Sensor_Data {
  int distanceFront;
  int distanceLeft;
  int distanceRight;
  float temperature;
  float pressure;
  int gasValue;
};

Sensor_Data sensorData;
bool controlMode = true;  
void setup() {
  Serial.begin(9600);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(A3);

  
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }

  
  radio.begin();
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));

    if (data.buttonPressed) {
      controlMode = !controlMode;
      delay(500);
    }

    if (controlMode) {
      
      handleMotorControl();
    } else {
      
      stopMotors();
      readSensorsAndTransmitData();
    }
  }
}

void handleMotorControl() {
  if (data.xAxis < 400) {
    
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (data.xAxis > 524) {
    
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else if (data.yAxis < 400) {
    
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (data.yAxis > 550) {
    
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    
    stopMotors();
  }
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void readSensorsAndTransmitData() {
  
  myServo.write(90); 
  delay(500);
  sensorData.distanceFront = measureDistance();
  
  myServo.write(0);
  delay(500);
  sensorData.distanceLeft = measureDistance();
  
  myServo.write(180); 
  delay(500);
  sensorData.distanceRight = measureDistance();
  
  /
  sensors_event_t event;
  bmp.getEvent(&event);
  bmp.getTemperature(&sensorData.temperature);
  sensorData.pressure = event.pressure;
  
  
  sensorData.gasValue = analogRead(mq2Pin);
  
  
  radio.stopListening();
  radio.write(&sensorData, sizeof(Sensor_Data));
  radio.startListening();
  
  
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

int measureDistance() {
  long duration, distance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}
