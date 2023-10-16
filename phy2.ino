//#include <Arduino.h>
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

struct Robot {
    int x, y;
    double true_x, true_y;
    int direction;
    int angle;
    int currentSpeed;
    int width, height;
    int auto_mode;
};

Robot myRobot;

void setup() {
    pinMode(irReceiverPin, INPUT);  // IR receiver pin is an input
    pinMode(irLedPin, OUTPUT);  // IR LED pin is an ouput
    pinMode(redLedPin, OUTPUT);  // Red LED pin is an output
    Serial.begin(9600);  // Set data rate to 9600 bps
}

void loop() {
    int leftDistance = irDistance(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN);
    int rightDistance = irDistance(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN);
    int centerDistance = irDistance(CENTER_IR_LED_PIN, CENTER_IR_SENSOR_PIN);

    // Debugging print
    Serial.print("Left: ");
    Serial.print(leftDistance);
    Serial.print(" | Center: ");
    Serial.print(centerDistance);
    Serial.print(" | Right: ");
    Serial.println(rightDistance);

    // Run the navigation function
    robotAutoMotorMove(&myRobot, centerDistance, leftDistance, rightDistance);

    // Implement the motor control based on the navigation decisions
    controlMotorsBasedOnNavigation(&myRobot);
}

#include <math.h>
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define CONSTANT_SPEED 10
#define ADJUSTMENT_SPEED 1  // Speed when adjusting
#define TURN_SPEED 1
#define MAX_SMOOTH_TURN_ANGLE 10 // Adjust as per testing
#define ADJUSTMENT_ANGLE 10  
#define TURN_DELAY_CYCLES 3  // Number of cycles to wait before initiating a turn
#define DEAD_ZONE 3  // Angle misalignment below which corrections are not made
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#define SMOOTHING_ANGLE 5  // Smaller angle for smoother adjustments
#define TURN_BASE 2  // Base for exponential increase in turn angle

void robotAutoMotorMove(struct Robot * robot, int front_centre_sensor, int left_sensor, int right_sensor) {
    static int adjustment_made = 0; 
    static int total_turn_angle = 0;  
    static int left_turn_count = 0;  // Counter for consecutive left turn detections
    static int right_turn_count = 0;  // Counter for consecutive right turn detections
    static int in_gradual_turn = 0;  // Flag to indicate whether the robot is in the middle of a gradual turn
    static int turning_left = 0;  // Flag to indicate whether the robot is turning left
    static int turning_right = 0;  // Flag to indicate whether the robot is turning right
    static int was_adjusted = 0;  // Flag to indicate if the robot was adjusted in the previous cycle
    static int is_completing_turn = 0;  // Flag to indicate if the robot is in the middle of completing a turn
    


    // If the front center sensor doesn't detect a wall, continue moving forward
    else {

        robot->direction = UP;
        robot->currentSpeed = CONSTANT_SPEED; 

        // If the robot is not centered and not currently turning, adjust the angle
        if (adjustment_made != 0 && !turning_left && !turning_right) {

            robot->angle -= adjustment_made;  
            adjustment_made = 0;  
        }

        // Logic for initiating a gradual left turn
        if (left_sensor == 0 && !turning_right) {

            turning_left = 1; 
            robot->currentSpeed = TURN_SPEED;
            if (total_turn_angle < 90) {
                // Linear increase for slower ramp-up
                int dynamic_smooth_turn_angle = MAX_SMOOTH_TURN_ANGLE + (0.1 * total_turn_angle);  
                dynamic_smooth_turn_angle = min(dynamic_smooth_turn_angle, 90 - total_turn_angle);  // Limit the maximum turn angle

                robot->angle -= dynamic_smooth_turn_angle;  
                total_turn_angle += dynamic_smooth_turn_angle;  
            }
            else {
                total_turn_angle = 0;  // Reset after completing the turn
                
                turning_left = 0;  
            }
        }
        // Logic for initiating a gradual right turn
        else if (right_sensor == 0 && !turning_left) {

            turning_right = 1;  
            robot->currentSpeed = TURN_SPEED;
            if (total_turn_angle < 90) {
                // Linear increase for slower ramp-up
                int dynamic_smooth_turn_angle = MAX_SMOOTH_TURN_ANGLE + (0.1 * total_turn_angle);  
                dynamic_smooth_turn_angle = min(dynamic_smooth_turn_angle, 90 - total_turn_angle);  // Limit the maximum turn angle

                robot->angle += dynamic_smooth_turn_angle;  
                total_turn_angle += dynamic_smooth_turn_angle;  
            }
            else {
                total_turn_angle = 0;  // Reset after completing the turn
                turning_right = 0;  
            }
        }


        // Stick to the wall logic
        if (left_sensor > 3 && left_sensor <= 5 && !was_adjusted) {

            robot->angle += ADJUSTMENT_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor > 3 && right_sensor <= 5 && !was_adjusted) {

            robot->angle -= ADJUSTMENT_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (left_sensor == 2 && !was_adjusted) {

            robot->angle -= SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor == 2 && !was_adjusted) {

            robot->angle += SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (left_sensor == 3 && !was_adjusted) {

            robot->angle += SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor == 3 && !was_adjusted) {

            robot->angle -= SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        }
        else {

            was_adjusted = 0;  // Reset adjustment flag
            robot->currentSpeed = CONSTANT_SPEED;  // Ensure to revert back to normal speed
        }
    }

}

#define SERVO_STOP 90
#define SERVO_MAX_FORWARD 180
#define SERVO_MAX_BACKWARD 0

#define MAX_TURN_SPEED_DIFFERENCE 50  // Max speed difference between wheels during a turn

void controlMotorsBasedOnNavigation(Robot* robot) {
    int leftServoSpeed = SERVO_STOP;
    int rightServoSpeed = SERVO_STOP;

    // Basic Movement: 
    // Map the speed [-1, 1] to servo command [SERVO_MAX_BACKWARD, SERVO_MAX_FORWARD]
    if(robot->currentSpeed != 0) {
        leftServoSpeed = map(robot->currentSpeed, -1, 1, SERVO_MAX_BACKWARD, SERVO_MAX_FORWARD);
        rightServoSpeed = map(robot->currentSpeed, -1, 1, SERVO_MAX_BACKWARD, SERVO_MAX_FORWARD);
    }

    // Turning adjustments while moving:
    if(robot->currentSpeed != 0) {
        int turnAdjustment = map(abs(robot->angle), 0, 90, 0, MAX_TURN_SPEED_DIFFERENCE);
        if(robot->angle > 0) {
            // Right Turn: Slow down the right wheel
            rightServoSpeed = constrain(rightServoSpeed - turnAdjustment, SERVO_MAX_BACKWARD, SERVO_MAX_FORWARD);
        } else if(robot->angle < 0) {
            // Left Turn: Slow down the left wheel
            leftServoSpeed = constrain(leftServoSpeed - turnAdjustment, SERVO_MAX_BACKWARD, SERVO_MAX_FORWARD);
        }
    }

    // Write the determined values to the servos
    servoLeft.write(leftServoSpeed);
    servoRight.write(rightServoSpeed);
}

int irDetect(long frequency) {
    tone(irLedPin, frequency);  // Turn on the IR LED square wave
    delay(1);  // Wait 1 ms
    noTone(irLedPin);  // Turn off the IR LED
    int ir = digitalRead(irReceiverPin);  // IR receiver -> ir variable
    delay(1);  // Down time before recheck
    return ir;  // Return 1 no detect, 0 detect
}

int irDistance(int irLedPin, int irReceiverPin) {
    int distance = 0;
    for(long f = 38000; f <= 42000; f += 1000) {
        distance += irDetect(f);
    }
    // Map the accumulated distance (which goes from 0 to 5) to a discrete scale (0 - 5).
    // Note: Actual mapping might need to be adjusted based on empirical testing.
    int scaledDistance = map(distance, 0, 50, 0, 5);  // Assuming 50 is a max value
    
    // Ensure that the scaled distance is within the desired range [0, 5].
    scaledDistance = constrain(scaledDistance, 0, 5);
    
    return scaledDistance;
}



