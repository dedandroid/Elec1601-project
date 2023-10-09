#include <Servo.h>

Servo servoLeft;
Servo servoRight;

const int LEFT_IR_LED_PIN = 10;
const int LEFT_IR_SENSOR_PIN = 11;
const int LEFT_RED_LED_PIN = A2;

const int RIGHT_IR_LED_PIN = 2;
const int RIGHT_IR_SENSOR_PIN = 3;
const int RIGHT_RED_LED_PIN = A0;

const int CENTER_IR_LED_PIN = 6;
const int CENTER_IR_SENSOR_PIN = 7;
const int CENTER_RED_LED_PIN = A1;


void setup() {
    servoLeft.attach(13);
    servoRight.attach(12);

    pinMode(LEFT_IR_LED_PIN, OUTPUT);
    pinMode(LEFT_IR_SENSOR_PIN, INPUT);
    pinMode(LEFT_RED_LED_PIN, OUTPUT);

    pinMode(RIGHT_IR_LED_PIN, OUTPUT);
    pinMode(RIGHT_IR_SENSOR_PIN, INPUT);
    pinMode(RIGHT_RED_LED_PIN, OUTPUT);

    pinMode(CENTER_IR_LED_PIN, OUTPUT);
    pinMode(CENTER_IR_SENSOR_PIN, INPUT);
    pinMode(CENTER_RED_LED_PIN, OUTPUT);

    delay(5000);
    moveForwardMedium();
  	Serial.begin(9600);
}

void loop() {
    int leftDistance = irDistance(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN);
    int rightDistance = irDistance(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN);
    int centerDistance = irDistance(CENTER_IR_LED_PIN, CENTER_IR_SENSOR_PIN);
  
    Serial.print("Left: ");
    Serial.print(leftDistance);
    Serial.print(" | Center: ");
    Serial.print(centerDistance);
    Serial.print(" | Right: ");
    Serial.println(rightDistance);

    if (centerDistance == 0) {
        digitalWrite(CENTER_RED_LED_PIN, HIGH);
        if (leftDistance > rightDistance) {
            turnLeftSlow();
        } else {
            turnRightSlow();
        }
        delay(3000);
        moveForwardMedium();
    } else {
        digitalWrite(CENTER_RED_LED_PIN, LOW);
    }

    if (rightDistance == 0) {
        digitalWrite(RIGHT_RED_LED_PIN, HIGH);
        turnRightSlow();
        delay(3000);
        moveForwardMedium();
    } else {
        digitalWrite(RIGHT_RED_LED_PIN, LOW);
    }

    if (leftDistance == 0) {
        digitalWrite(LEFT_RED_LED_PIN, HIGH);
        turnLeftSlow();
        delay(3000);
        moveForwardMedium();
    } else {
        digitalWrite(LEFT_RED_LED_PIN, LOW);
    }
}

int irDetect(int ledPin, int receivePin, long frequency) {
    tone(ledPin, frequency);
    delay(1);
    noTone(ledPin);
    int ir = digitalRead(receivePin);
    delay(1);
    if (ir == 0) {
        return frequency; // Return the frequency where an object was detected
    } else {
        return 0; // Return 0 if no object was detected at this frequency
    }
}

int irDistance(int irLedPin, int irReceivePin) {
   for(long f = 38000; f <= 42000; f += 1000) {
      int detectedFrequency = irDetect(irLedPin, irReceivePin, f);
      if (detectedFrequency > 0) {
          return detectedFrequency; // Return the first detected frequency
      }
   }
   return 0; // Return 0 if no object was detected across all tested frequencies
}

void moveForwardMedium() {
    servoLeft.writeMicroseconds(1600);
    servoRight.writeMicroseconds(1400);
}

void turnRightSlow() {
    servoLeft.writeMicroseconds(1550);
    servoRight.writeMicroseconds(1550);
}

void turnLeftSlow() {
    servoLeft.writeMicroseconds(1450);
    servoRight.writeMicroseconds(1450);
}