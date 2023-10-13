#include "robot.h"
#include <SDL2/SDL_image.h>

void setup_robot(struct Robot *robot){
    robot->x = OVERALL_WINDOW_WIDTH/2-50;//71;
    robot->y = OVERALL_WINDOW_HEIGHT-50;//76;
    robot->true_x = OVERALL_WINDOW_WIDTH/2-50;//71;
    robot->true_y = OVERALL_WINDOW_HEIGHT-50;//76;
    robot->width = ROBOT_WIDTH;
    robot->height = ROBOT_HEIGHT;
    robot->direction = 0;
    robot->angle = 0;
    robot->currentSpeed = 0;
    robot->crashed = 0;
    robot->auto_mode = 0;

    printf("Press arrow keys to move manually, or enter to move automatically\n\n");
}
int robot_off_screen(struct Robot * robot){
    if(robot->x < 0 || robot-> y < 0){
        return 0;
    }
    if(robot->x > OVERALL_WINDOW_WIDTH || robot->y > OVERALL_WINDOW_HEIGHT){
        return 0;
    }
    return 1;
}

int checkRobotHitWall(struct Robot * robot, struct Wall * wall) {

    int overlap = checkOverlap(robot->x,robot->width,robot->y,robot->height,
                 wall->x,wall->width,wall->y, wall->height);

    return overlap;
}

int checkRobotHitWalls(struct Robot * robot, struct Wall_collection * head) {
   struct Wall_collection *ptr = head;
   int hit = 0;

   while(ptr != NULL) {
      hit = (hit || checkRobotHitWall(robot, &ptr->wall));
      ptr = ptr->next;
   }
   return hit;

}

int checkRobotReachedEnd(struct Robot * robot, int x, int y, int width, int height){

    int overlap = checkOverlap(robot->x,robot->width,robot->y,robot->height,
                 x,width,y,height);

    return overlap;
}

void robotCrash(struct Robot * robot) {
    robot->currentSpeed = 0;
    if (!robot->crashed)
        printf("Ouchies!!!!!\n\nPress space to start again\n");
    robot->crashed = 1;
}

void robotSuccess(struct Robot * robot, int msec) {
    robot->currentSpeed = 0;
    if (!robot->crashed){
        printf("Success!!!!!\n\n");
        printf("Time taken %d seconds %d milliseconds \n", msec/1000, msec%1000);
        printf("Press space to start again\n");
    }
    robot->crashed = 1;
}

int checkRobotSensor(int x, int y, int sensorSensitivityLength, struct Wall * wall)  {
    //viewing_region of sensor is a square of 2 pixels * chosen length of sensitivity
    int overlap = checkOverlap(x,2,y,sensorSensitivityLength,
                 wall->x,wall->width,wall->y, wall->height);

    return overlap;
}

int checkRobotSensorFrontCentreAllWalls(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;

    int sensorSensitivityLength =  floor(SENSOR_VISION/5);

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        //xDir = round(robotCentreX+(ROBOT_WIDTH/2-2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle)*PI/180));
        //yDir = round(robotCentreY+(ROBOT_WIDTH/2-2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle)*PI/180));
        xDir = round(robotCentreX-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle)*PI/180));
        yDir = round(robotCentreY+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

int checkRobotSensorLeftAllWalls(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;
    sensorSensitivityLength =  floor(SENSOR_VISION/5);

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        //xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle)*PI/180));
        //yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle)*PI/180));
        xDir = round(robotCentreX+(+ROBOT_WIDTH/2)*cos((robot->angle-90)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle-90)*PI/180));
        yDir = round(robotCentreY+(+ROBOT_WIDTH/2)*sin((robot->angle-90)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle-90)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

int checkRobotSensorRightAllWalls(struct Robot * robot, struct Wall_collection * head) {
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength;

    head_store = head;
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;
    score = 0;
    sensorSensitivityLength =  floor(SENSOR_VISION/5);

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        //xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle)*PI/180));
        //yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle)*PI/180));
        xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle+90)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*sin((robot->angle+90)*PI/180));
        yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle+90)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensorSensitivityLength*i)*cos((robot->angle+90)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;
        hit = 0;

        while(ptr != NULL) {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

void robotUpdate(struct SDL_Renderer * renderer, struct Robot * robot){
    double xDir, yDir;

    int robotCentreX, robotCentreY, xTR, yTR, xTL, yTL, xBR, yBR, xBL, yBL;
    //SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);

    /*
    //Other Display options:
    // The actual square which the robot is tested against (not so nice visually with turns, but easier
    // to test overlap
    SDL_Rect rect = {robot->x, robot->y, robot->height, robot->width};
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &rect);
    SDL_RenderFillRect(renderer, &rect);
    */
    /*
    //Center Line of Robot. Line shows the direction robot is facing
    xDir = -30 * sin(-robot->angle*PI/180);
    yDir = -30 * cos(-robot->angle*PI/180);
    xDirInt = robot->x+ROBOT_WIDTH/2+ (int) xDir;
    yDirInt = robot->y+ROBOT_HEIGHT/2+ (int) yDir;
    SDL_RenderDrawLine(renderer,robot->x+ROBOT_WIDTH/2, robot->y+ROBOT_HEIGHT/2, xDirInt, yDirInt);
    */
    // Load the car image
    if ( !( IMG_Init(IMG_INIT_PNG) & IMG_INIT_JPG ) ) {
        printf("Could not initialize SDL_image \n");
    }
    SDL_Surface *carImage = IMG_Load("white_car.png"); // Replace with the actual image file path
    if (carImage != NULL) {
        // Create a texture from the original image
        SDL_Texture *carTexture = SDL_CreateTextureFromSurface(renderer, carImage);
        SDL_FreeSurface(carImage);

        // Set the destination rectangle for the car image
        SDL_Rect carDestinationRect;
        carDestinationRect.x = robot->true_x - ROBOT_WIDTH + (5 - 5/22 *(ROBOT_WIDTH - 42));
        carDestinationRect.y = robot->true_y;
        carDestinationRect.w = ROBOT_HEIGHT*2;
        carDestinationRect.h = ROBOT_WIDTH *2;

        // Rotate and render the car image
        double angleRadians = robot->angle * M_PI / 180.0;
        SDL_RenderCopyEx(renderer, carTexture, NULL, &carDestinationRect, angleRadians * 180.0 / M_PI, NULL, SDL_FLIP_NONE);

        // Destroy the car texture when done
        SDL_DestroyTexture(carTexture);
    }

    //Rotating Square
    //Vector rotation to work out corners x2 = x1cos(angle)-y1sin(angle), y2 = x1sin(angle)+y1cos(angle)
    robotCentreX = robot->x+ROBOT_WIDTH/2;
    robotCentreY = robot->y+ROBOT_HEIGHT/2;

    xDir = round(robotCentreX+(ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2)*sin((robot->angle)*PI/180));
    yDir = round(robotCentreY+(ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2)*cos((robot->angle)*PI/180));
    xTR = (int) xDir;
    yTR = (int) yDir;

    xDir = round(robotCentreX+(ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(ROBOT_HEIGHT/2)*sin((robot->angle)*PI/180));
    yDir = round(robotCentreY+(ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(ROBOT_HEIGHT/2)*cos((robot->angle)*PI/180));
    xBR = (int) xDir;
    yBR = (int) yDir;

    xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(ROBOT_HEIGHT/2)*sin((robot->angle)*PI/180));
    yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(ROBOT_HEIGHT/2)*cos((robot->angle)*PI/180));
    xBL = (int) xDir;
    yBL = (int) yDir;

    xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2)*sin((robot->angle)*PI/180));
    yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2)*cos((robot->angle)*PI/180));
    xTL = (int) xDir;
    yTL = (int) yDir;

    SDL_RenderDrawLine(renderer,xTR, yTR, xBR, yBR);
    SDL_RenderDrawLine(renderer,xBR, yBR, xBL, yBL);
    SDL_RenderDrawLine(renderer,xBL, yBL, xTL, yTL);
    SDL_RenderDrawLine(renderer,xTL, yTL, xTR, yTR);

    //Front Centre Sensor
    int sensor_sensitivity =  floor(SENSOR_VISION/5);
    int i;
    for (i = 0; i < 5; i++)
    {
        xDir = round(robotCentreX-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*sin((robot->angle)*PI/180));
        yDir = round(robotCentreY+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*cos((robot->angle)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_Rect rect = {xTL, yTL, 2, sensor_sensitivity};
        SDL_SetRenderDrawColor(renderer, 80+(20*(5-i)), 80+(20*(5-i)), 80+(20*(5-i)), 255);
        SDL_RenderDrawRect(renderer, &rect);
        SDL_RenderFillRect(renderer, &rect);
    }

    //Left Sensor
    for (i = 0; i < 5; i++)
    {
        xDir = round(robotCentreX+(+ROBOT_WIDTH/2)*cos((robot->angle-90)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*sin((robot->angle-90)*PI/180));
        yDir = round(robotCentreY+(+ROBOT_WIDTH/2)*sin((robot->angle-90)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*cos((robot->angle-90)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_Rect rect = {xTL, yTL, 2, sensor_sensitivity};
        SDL_SetRenderDrawColor(renderer, 80+(20*(5-i)), 80+(20*(5-i)), 80+(20*(5-i)), 255);
        SDL_RenderDrawRect(renderer, &rect);
        SDL_RenderFillRect(renderer, &rect);
    }

    //Right Sensor
    for (i = 0; i < 5; i++)
    {
        xDir = round(robotCentreX+(-ROBOT_WIDTH/2)*cos((robot->angle+90)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*sin((robot->angle+90)*PI/180));
        yDir = round(robotCentreY+(-ROBOT_WIDTH/2)*sin((robot->angle+90)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*cos((robot->angle+90)*PI/180));
        xTL = (int) xDir;
        yTL = (int) yDir;

        SDL_Rect rect = {xTL, yTL, 2, sensor_sensitivity};
        SDL_SetRenderDrawColor(renderer, 80+(20*(5-i)), 80+(20*(5-i)), 80+(20*(5-i)), 255);
        SDL_RenderDrawRect(renderer, &rect);
        SDL_RenderFillRect(renderer, &rect);
    }

        //xDir = round(robotCentreX+(ROBOT_WIDTH/2-2)*cos((robot->angle)*PI/180)-(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*sin((robot->angle)*PI/180));
        //yDir = round(robotCentreY+(ROBOT_WIDTH/2-2)*sin((robot->angle)*PI/180)+(-ROBOT_HEIGHT/2-SENSOR_VISION+sensor_sensitivity*i)*cos((robot->angle)*PI/180));
}



void robotMotorMove(struct Robot * robot, int crashed) {
    double x_offset, y_offset;
    if (crashed)
        robot->currentSpeed = 0;
    else {
        switch(robot->direction){
            case UP :
                robot->currentSpeed += DEFAULT_SPEED_CHANGE;
                if (robot->currentSpeed > MAX_ROBOT_SPEED)
                    robot->currentSpeed = MAX_ROBOT_SPEED;
                break;
            case DOWN :
                robot->currentSpeed -= DEFAULT_SPEED_CHANGE;
                if (robot->currentSpeed < -MAX_ROBOT_SPEED)
                    robot->currentSpeed = -MAX_ROBOT_SPEED;
                break;
            case LEFT :
                robot->angle = (robot->angle+360-DEFAULT_ANGLE_CHANGE)%360;
                break;
            case RIGHT :
                robot->angle = (robot->angle+DEFAULT_ANGLE_CHANGE)%360;
                break;
        }
    }
    robot->direction = 0;
    x_offset = (-robot->currentSpeed * sin(-robot->angle*PI/180));
    y_offset = (-robot->currentSpeed * cos(-robot->angle*PI/180));

    robot->true_x += x_offset;
    robot->true_y += y_offset;

    x_offset = round(robot->true_x);
    y_offset = round(robot->true_y);

    robot->x = (int) x_offset;
    robot->y = (int) y_offset;
}
#include <math.h>

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
    
    // If the robot is completing a turn, ignore all sensors and finish the turn
    if (is_completing_turn) {
        printf("Completing a turn...\n");
        int remaining_angle = 90 - total_turn_angle;  
        robot->angle += turning_left ? -remaining_angle : remaining_angle;  
        total_turn_angle = 0;
        turning_left = turning_right = 0;
        is_completing_turn = 0;
        return;
    }

    // If the front center sensor detects a wall
    if (front_centre_sensor != 0) {
        printf("Wall detected in front!\n");
        robot->currentSpeed = 0;  
        
        // New logic for initiating a turn when a wall is detected:
        if (right_sensor == 0) {
            printf("Turning right (wall in front, no wall to right)...\n");
            robot->angle += 90;  // Make a 90-degree right turn
            return;  // Exit function after initiating the turn
        }
        else if (left_sensor == 0) {
            printf("Turning left (wall in front, no wall to left)...\n");
            robot->angle -= 90;  // Make a 90-degree left turn
            return;  // Exit function after initiating the turn
        }

        else if (left_sensor == 1 && left_sensor == 1 )  {
            printf("Turning back)...\n");
            robot->angle -= 180;  // Make a 90-degree left turn
            return;  // Exit function after initiating the turn
        }

        // Complete the turn if already in progress
        if (turning_left || turning_right) {
            printf("Completing a turn (wall in front, turning left/right)...\n");
            is_completing_turn = 1;  // Set the flag to ignore sensors until the turn is complete
            return;
        }
    }

    // If the front center sensor doesn't detect a wall, continue moving forward
    else {
        printf("No wall in front, moving forward...\n");
        robot->direction = UP;
        robot->currentSpeed = CONSTANT_SPEED; 

        // If the robot is not centered and not currently turning, adjust the angle
        if (adjustment_made != 0 && !turning_left && !turning_right) {
            printf("Adjusting angle (not centered, not turning)...\n");
            robot->angle -= adjustment_made;  
            adjustment_made = 0;  
        }

        // Logic for initiating a gradual left turn
        if (left_sensor == 0 && !turning_right) {
            printf("Initiating gradual left turn...\n");
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
            printf("Initiating gradual right turn...\n");
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
            printf("Adjusting left (left sensor: %d)...\n", left_sensor);
            robot->angle += ADJUSTMENT_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor > 3 && right_sensor <= 5 && !was_adjusted) {
            printf("Adjusting right (right sensor: %d)...\n", right_sensor);
            robot->angle -= ADJUSTMENT_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (left_sensor == 2 && !was_adjusted) {
            printf("Smoothing left (left sensor: 2)...\n");
            robot->angle -= SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor == 2 && !was_adjusted) {
            printf("Smoothing right (right sensor: 2)...\n");
            robot->angle += SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (left_sensor == 3 && !was_adjusted) {
            printf("Smoothing left (left sensor: 3)...\n");
            robot->angle += SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        } 
        else if (right_sensor == 3 && !was_adjusted) {
            printf("Smoothing right (right sensor: 3)...\n");
            robot->angle -= SMOOTHING_ANGLE;
            robot->currentSpeed = ADJUSTMENT_SPEED;  // Slow down when adjusting
            was_adjusted = 1;
        }
        else {
            printf("No adjustment needed, moving forward...\n");
            was_adjusted = 0;  // Reset adjustment flag
            robot->currentSpeed = CONSTANT_SPEED;  // Ensure to revert back to normal speed
        }
    }
}



























