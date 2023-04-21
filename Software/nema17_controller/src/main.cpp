#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "AS5600.h"
#include "Adafruit_INA219.h"
#include "AccelStepper.h"

// Led pins
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
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// I2C bus
#define SDA_PIN 0
#define SCL_PIN 1

// Current sensor INA219AIDCNR. We use 0.01 Ohm shunt resistor but in the lib is 0.1 Ohm
Adafruit_INA219 ina219;
#define CURRENT_SENSOR_ADDR 0x40



// Functions Prototypes
void encoderRead(void *parameter);
void ledFlash(void *parameter);


void setup() {

  // Start serial
  Serial.begin(115200);
  delay(1000); // for serial monitor
  Serial.println("Starting...");

  // Setup Led pins
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, HIGH);

  // Create i2c object
  Wire.begin(SDA_PIN, SCL_PIN); // Start I2C bus with pins SDA_PIN and SCL_PIN. The ICs used Wire default Wire object.

  // Start encoder
  as5600.begin(SDA_PIN, SCL_PIN, 255); // as5600 start his own i2c bus kinda. 255 is for software direction.
  if(as5600.isConnected()){
    Serial.println("Encoder connected");
    xTaskCreatePinnedToCore(encoderRead, "encoderRead", 10000, NULL, 1, NULL, 0);
  } 

  // Start Current Sensor
  if(!ina219.begin()){
    Serial.println("Failed to find INA219 chip");
  }

  // Start stepper
  stepper.setEnablePin(EN_PIN);
  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(50.0);
  stepper.disableOutputs();

  

  // Start task
  xTaskCreatePinnedToCore(ledFlash, "ledFlash", 10000, NULL, 1, NULL, 0);
}

void loop() {
  
}


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