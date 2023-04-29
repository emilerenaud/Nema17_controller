#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "AS5600.h"
#include "Adafruit_INA219.h"
#include "AccelStepper.h"
#include "controller.h"
#include "PCB_config.h"



// Controller object
Controller controller;


// Functions Prototypes
void encoderRead(void *parameter);
void ledFlash(void *parameter);
void processSerialCommands();

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
  xTaskCreatePinnedToCore(ledFlash, "ledFlash", 10000, NULL, 1, NULL, 0);

  controller.init();


}

void loop() {
  processSerialCommands();
  controller.run();
}


void ledFlash(void *parameter) {
  while(1){
    digitalWrite(LED_1, !digitalRead(LED_1));
    digitalWrite(LED_2, !digitalRead(LED_2));
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Serial.println(ina219.getPower_mW());
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    // read the incoming byte
    if (Serial.read() == '$') {
      // start of command detected
      String command = "";
      command = Serial.readStringUntil('\n');
      // decode command
      // find colon character
      int colonIndex = command.indexOf(':');
      if (colonIndex != -1) {
        // colon character found
        String commandName = command.substring(0, colonIndex);
        String commandValue = command.substring(colonIndex + 1);
        // decode command
        if (commandName == "moveTo") {
          int value = commandValue.toInt();
          Serial.println("Moving to " + String(value));
          controller.moveStep(value);
          // do something with value
        }
        else {
          // unsupported command
        }
      }
    }
  }
}
