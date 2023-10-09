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

    // Start by moving forward at medium speed after 5 seconds
    delay(5000);
    moveForwardMedium();
}


void loop() {
    int leftDistance = irDistance(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN);
    int rightDistance = irDistance(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN);
    int centerDistance = irDistance(CENTER_IR_LED_PIN, CENTER_IR_SENSOR_PIN);

    // Handle the center detection and LED
    if (centerDistance == 0) {
        digitalWrite(CENTER_RED_LED_PIN, HIGH); // Turn on the center red LED
        if (leftDistance > rightDistance) {
            turnLeftSlow();
        } else {
            turnRightSlow();
        }
        delay(3000); // Wait for 3 seconds
        moveForwardMedium();
    } else {
        digitalWrite(CENTER_RED_LED_PIN, LOW); // Turn off the center red LED
    }

    // Handle the right detection and LED
    if (rightDistance == 0) {
        digitalWrite(RIGHT_RED_LED_PIN, HIGH); // Turn on the right red LED
        turnRightSlow();
        delay(3000); // Wait for 3 seconds
        moveForwardMedium();
    } else {
        digitalWrite(RIGHT_RED_LED_PIN, LOW); // Turn off the right red LED
    }

    // Handle the left detection and LED
    if (leftDistance == 0) {
        digitalWrite(LEFT_RED_LED_PIN, HIGH); // Turn on the left red LED
        turnLeftSlow();
        delay(3000); // Wait for 3 seconds
        moveForwardMedium();
    } else {
        digitalWrite(LEFT_RED_LED_PIN, LOW); // Turn off the left red LED
    }
}


int irDetect(int ledPin, int receivePin, long frequency) {
    tone(ledPin, frequency);
    delay(1);
    noTone(ledPin);
    int ir = digitalRead(receivePin);
    delay(1);
    return ir;
}

int irDistance(int irLedPin, int irReceivePin) {
   int distance = 0;
   for(long f = 38000; f <= 42000; f += 1000) {
      distance += irDetect(irLedPin, irReceivePin, f);
   }
   return distance;
}

void moveForwardMedium() {
    servoLeft.writeMicroseconds(1600);  // Counter-clockwise (medium speed)
    servoRight.writeMicroseconds(1400); // Clockwise (medium speed)
}

void turnRightSlow() {
    servoLeft.writeMicroseconds(1550);  // Counter-clockwise (slow speed)
    servoRight.writeMicroseconds(1550); // Clockwise (slow speed)
}

void turnLeftSlow() {
    servoLeft.writeMicroseconds(1450);  // Counter-clockwise (slow speed)
    servoRight.writeMicroseconds(1450); // Clockwise (slow speed)
}