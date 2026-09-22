/*
 * L298N Motor Test
 * Controls 4 motors using PWM and direction pins.
 */

#include <Arduino.h>

// Define pin connections
#define ENA 19
#define IN1 18
#define IN2 5

#define IN3 17
#define IN4 16
#define ENB 4

void testMotorSetup()
{
   // Set motor control pins as outputs
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
   pinMode(ENA, OUTPUT);
   pinMode(ENB, OUTPUT);

   // Initialize Serial Monitor
   Serial.begin(115200);
   Serial.println("L298N Motor Test Initialized");
}

void testMotorLoop()
{
   // Test Motor A (connected to IN1, IN2, ENA)
   Serial.println("Motor A Forward");
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW);
   analogWrite(ENA, 255); // 50% speed
   delay(2000);

   Serial.println("Motor A Reverse");
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, HIGH);
   analogWrite(ENA, 255); // 50% speed
   delay(2000);

   // Stop Motor A
   Serial.println("Motor A Stop");
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, LOW);
   analogWrite(ENA, 0);
   delay(1000);

   // Test Motor B (connected to IN3, IN4, ENB)
   Serial.println("Motor B Forward");
   digitalWrite(IN3, HIGH);
   digitalWrite(IN4, LOW);
   analogWrite(ENB, 255); // 50% speed
   delay(2000);

   Serial.println("Motor B Reverse");
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);
   analogWrite(ENB, 255); // 50% speed
   delay(2000);

   // Stop Motor B
   Serial.println("Motor B Stop");
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, LOW);
   analogWrite(ENB, 0);
   delay(1000);
}