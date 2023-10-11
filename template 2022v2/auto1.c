#define CONSTANT_SPEED 5 
#define TURN_SPEED 1
#define MAX_SMOOTH_TURN_ANGLE 6  // Adjust as per testing
#define ADJUSTMENT_ANGLE 5  
#define TURN_DELAY_CYCLES 3  // Number of cycles to wait before initiating a turn
#define DEAD_ZONE 3  // Angle misalignment below which corrections are not made
#define max(a,b) ((a) > (b) ? (a) : (b))

void robotAutoMotorMove(struct Robot * robot, int front_centre_sensor, int left_sensor, int right_sensor) {
    static int adjustment_made = 0; 
    static int total_turn_angle = 0;  
    static int left_turn_count = 0;  // Counter for consecutive left turn detections
    static int right_turn_count = 0;  // Counter for consecutive right turn detections
    static int in_gradual_turn = 0;  // Flag to indicate whether the robot is in the middle of a gradual turn
    static int turning_left = 0;  // Flag to indicate whether the robot is turning left
    static int turning_right = 0;  // Flag to indicate whether the robot is turning right
    
    // If the front center sensor detects a wall
    if (front_centre_sensor == 1) {
        robot->currentSpeed = 0;  
        
        // Complete the turn if already in progress
        if (turning_left || turning_right) {
            int remaining_angle = 90 - total_turn_angle;  
            robot->angle += turning_left ? -remaining_angle : remaining_angle;  
            turning_left = turning_right = 0;  
            total_turn_angle = 0;  
            return;
        }
        // Immediate decision-making regarding the direction
        else if (left_sensor == 0) {
            robot->angle -= 90;  
            return;
        }
        else if (right_sensor == 0) {
            robot->angle += 90;  
            return;
        }
        else {
            robot->angle += 180;  // U-turns can remain the same
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
                int dynamic_smooth_turn_angle = MAX_SMOOTH_TURN_ANGLE - sqrt(total_turn_angle);  
                dynamic_smooth_turn_angle = max(dynamic_smooth_turn_angle, 1);  

                robot->angle -= dynamic_smooth_turn_angle;  
                total_turn_angle += dynamic_smooth_turn_angle;  
            }
            else {
                total_turn_angle = 0;  
                turning_left = 0;  
            }
        }
        // Logic for initiating a gradual right turn
        else if (right_sensor == 0 && !turning_left) {
            turning_right = 1;  
            robot->currentSpeed = TURN_SPEED;
            if (total_turn_angle < 90) {
                int dynamic_smooth_turn_angle = MAX_SMOOTH_TURN_ANGLE - sqrt(total_turn_angle);  
                dynamic_smooth_turn_angle = max(dynamic_smooth_turn_angle, 1);  

                robot->angle += dynamic_smooth_turn_angle;  
                total_turn_angle += dynamic_smooth_turn_angle;  
            }
            else {
                total_turn_angle = 0;  
                turning_right = 0;  
            }
        }
        else {
            total_turn_angle = 0;  
            turning_left = turning_right = 0;  
        }


        // Re-centering logic
        if (left_sensor >= 4 && adjustment_made >= -ADJUSTMENT_ANGLE && adjustment_made > -DEAD_ZONE) {
            robot->angle += ADJUSTMENT_ANGLE;
            adjustment_made += ADJUSTMENT_ANGLE;
        } 
        else if (right_sensor >= 4 && adjustment_made <= ADJUSTMENT_ANGLE && adjustment_made < DEAD_ZONE) {
            robot->angle -= ADJUSTMENT_ANGLE;
            adjustment_made -= ADJUSTMENT_ANGLE;
        }
        else if (adjustment_made != 0 && abs(adjustment_made) > DEAD_ZONE) {
            robot->angle -= adjustment_made;  
            adjustment_made = 0;  
        }
    }
}


