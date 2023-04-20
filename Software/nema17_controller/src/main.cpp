#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "AS5600.h"
#include "Adafruit_INA219.h"

#define LED_1 3
#define LED_2 5

// Position encoder AS5600-ASOM
#define ENCODER_OUT ADC1_CHANNEL_4
#define ENCODER_ADDR 0x36
AS5600 as5600;

// Stepper pins A4899 driver
#define STEP_PIN 7
#define DIR_PIN 5
#define EN_PIN 10


// I2C bus
#define SDA_PIN 0
#define SCL_PIN 1

// Current sensor INA219AIDCNR. We use 0.01 Ohm shunt resistor but in the lib is 0.1 Ohm
Adafruit_INA219 ina219;
#define CURRENT_SENSOR_ADDR 0x40

// create task that make flash the LED each 1 second
void ledFlash(void *parameter) {
  while(1){
    digitalWrite(LED_1, !digitalRead(LED_1));
    digitalWrite(LED_2, !digitalRead(LED_2));
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Serial.println(ina219.getPower_mW());
  }
}

// Create task that read the encoder
void encoderRead(void *parameter) {
  while(1){
    Serial.println(as5600.getCumulativePosition());
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {

  // Start serial
  Serial.begin(115200);
  delay(1000); // for serial monitor
  Serial.println("Starting...");

  // put your setup code here, to run once:
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, HIGH);

  // Create i2c object
  Wire.begin(SDA_PIN, SCL_PIN);\

  // Start encoder
  as5600.begin(SDA_PIN, SCL_PIN, 255); // as5600 start his own i2c bus kinda. 255 is for software direction.

  if(as5600.isConnected()){
    Serial.println("Encoder connected");
    xTaskCreatePinnedToCore(encoderRead, "encoderRead", 10000, NULL, 1, NULL, 0);
  } 

  // Setup Current Sensor
  if(!ina219.begin()){
    Serial.println("Failed to find INA219 chip");
  }

  

  // Start task
  xTaskCreatePinnedToCore(ledFlash, "ledFlash", 10000, NULL, 1, NULL, 0);
}

void loop() {
  
}