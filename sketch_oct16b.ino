#include <Servo.h>

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

void setup()                                 // Built-in initialization block
{
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

   Serial.begin(9600);                       // Set data rate to 9600 bps
}  
 
void loop()                                  // Main loop auto-repeats
{
  int LeftirVal = irDetect(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN, 42000);               // Check for Left
  int leftDist = irDistance(LEFT_IR_LED_PIN, LEFT_IR_SENSOR_PIN);
  Serial.print("Left : ");
  Serial.println(leftDist);                     // Display 1/0 no detect/detect

  int RightirVal = irDetect(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN, 42000);               // Check for Left
  int rightDist = irDistance(RIGHT_IR_LED_PIN, RIGHT_IR_SENSOR_PIN);
  Serial.print("Right : ");
  Serial.println(rightDist);                     // Display 1/0 no detect/detect

  int CenterirVal = irDetect(CENTER_IR_LED_PIN, CENTER_IR_SENSOR_PIN, 42000);               // Check for Left
  int centerDist = irDistance(CENTER_IR_LED_PIN, CENTER_IR_SENSOR_PIN);
  Serial.print("Center : ");
  Serial.println(centerDist);                     // Display 1/0 no detect/detect

  // Run the navigation function
  robotAutoMotorMove(&myRobot, centerDist, leftDist,rightDist);

    // Implement the motor control based on the navigation decisions
  controlMotorsBasedOnNavigation(&myRobot);



  if (LeftirVal == 0)                            // Optional - display detection by setting red LED high
  {
    digitalWrite(LEFT_RED_LED_PIN, HIGH);             
  } else{
    digitalWrite(LEFT_RED_LED_PIN, LOW);
  }
  
  
  if (RightirVal == 0)                            // Optional - display detection by setting red LED high
  {
    digitalWrite(RIGHT_RED_LED_PIN, HIGH);             
  } else{
    digitalWrite(RIGHT_RED_LED_PIN, LOW);
  }

  if (CenterirVal == 0)                            // Optional - display detection by setting red LED high
  {
    digitalWrite(CENTER_RED_LED_PIN, HIGH);             
  } else{
    digitalWrite(CENTER_RED_LED_PIN, LOW);
  }

  delay(500);                                // 0.5 second delay - just long enough to see the LED blink
  // digitalWrite(LEFT_RED_LED_PIN, HIGH);
  // // delay(500);                                // 0.5 second delay - just long enough to see the LED blink
  // digitalWrite(CENTER_RED_LED_PIN, HIGH);
  // // delay(500);                                // 0.5 second delay - just long enough to see the LED blink
  // digitalWrite(RIGHT_RED_LED_PIN, HIGH);
  
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
    
    if(front_centre_sensor != 0) {
        digitalWrite(CENTER_RED_LED_PIN, HIGH);
    } else {
        digitalWrite(CENTER_RED_LED_PIN, LOW);
    }

    if(left_sensor != 0) {
        digitalWrite(LEFT_RED_LED_PIN, HIGH);
    } else {
        digitalWrite(LEFT_RED_LED_PIN, LOW);
    }

    if(right_sensor != 0) {
        digitalWrite(RIGHT_RED_LED_PIN, HIGH);
    } else {
        digitalWrite(RIGHT_RED_LED_PIN, LOW);
    }


    // If the robot is completing a turn, ignore all sensors and finish the turn
    if (is_completing_turn) {

        int remaining_angle = 90 - total_turn_angle;  
        robot->angle += turning_left ? -remaining_angle : remaining_angle;  
        total_turn_angle = 0;
        turning_left = turning_right = 0;
        is_completing_turn = 0;
        return;
    }

    // If the front center sensor detects a wall
    if (front_centre_sensor != 0) {

        robot->currentSpeed = 0;  
        
        // New logic for initiating a turn when a wall is detected:
        if (right_sensor == 0) {

            robot->angle += 90;  // Make a 90-degree right turn
            return;  // Exit function after initiating the turn
        }
        else if (left_sensor == 0) {

            robot->angle -= 90;  // Make a 90-degree left turn
            return;  // Exit function after initiating the turn
        }

        else if (left_sensor == 1 && left_sensor == 1 )  {

            robot->angle -= 180;  // Make a 90-degree left turn
            return;  // Exit function after initiating the turn
        }

        // Complete the turn if already in progress
        if (turning_left || turning_right) {

            is_completing_turn = 1;  // Set the flag to ignore sensors until the turn is complete
            return;
        }
    }

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


// IR Object Detection Function

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency);                 // Turn on the IR LED square wave
  delay(1);                                  // Wait 1 ms
  noTone(irLedPin);                          // Turn off the IR LED
  int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck

  
  if (ir == 0){
    return 1;
  }

  else{
    return 0;
  }
  // return ir;                                 // Return 0 detect, 1 no detect
}


// IR distance measurement function

int irDistance(int irLedPin, int irReceivePin)
{
   int distance = 0;  
   for(long f = 47000; f <= 50000; f += 1000)
   {
      int val = irDetect(irLedPin, irReceivePin, f);
      distance += val;
   }
   return distance;
}