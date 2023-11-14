//recording functions and testing
//LCD updates
//Inverse-Kinematics big fail did not get it to work


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27,16,4);
// Initialize servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Initialize Bluetooth module
SoftwareSerial bt1(2,3); /* (Rx,Tx) */

// Define servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000;
const int moveCount = 10;// Define maximum moves and servos
const int servoNumber = 6;

int servoAngles[6] = {135, 95, 105, 95, 170, 85}; // Initial angles for each servo
int movesServos[moveCount][servoNumber]; //max of 10 moves

bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

// Function Prototypes function must be declared before it is called unless it is defined above the point where it is called.
int pulseWidth(int angle);
void executeCommand(String command);
void printServoPulseWidths();
void MoveToPick();
void MoveToStart();
void moveServo(int servoChannel, int angle);
void StartRecordingMovements();
void StopRecordingMovements();
void PlayRecordedMovements();



struct Point {
    float x;
    float y;
    float z;
};

// Define the number of joints, including the end-effector
const int numJoints = 5; // Adjust if your arm has more joints

// Define the positions of the joints
Point joints[numJoints];
// Define the arm's segment lengths
const float lengths[] = {85, 105, 129, 70}; // Lengths in mm

// Define the positions of the joints (initialized to some starting configuration)

// Define the maximum iterations and tolerance for the IK solution
const int maxIterations = 10;
const float tolerance = 1.0; // 1 mm tolerance
float convertPositionToAngle(Point a, Point b);
float distanceBetween(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}


float calculateBaseAngle(Point target) {
    // Angle in the horizontal plane from the x-axis
    return atan2(target.y, target.x) * 180 / PI; // Convert to degrees
}



float calculatePlanarJointAngle(Point currentJoint, Point nextJoint) {
    // Project onto the vertical plane and calculate the angle
    float deltaX = nextJoint.x - currentJoint.x;
    float deltaZ = nextJoint.z - currentJoint.z; // Assuming Z is up/down axis
    return atan2(deltaZ, deltaX) * 180 / PI; // Convert to degrees
}

float calculateWristAngle(Point currentJoint, Point previousJoint) {
    // Calculate angle based on the relative position to the previous joint
    float deltaX = currentJoint.x - previousJoint.x;
    float deltaZ = currentJoint.z - previousJoint.z; // Assuming Z is up/down axis
    // This may need to account for the previous joint's rotation
    return atan2(deltaZ, deltaX) * 180 / PI; // Convert to degrees
}


void applyIKSolutionToRobot(Point joints[]) {
    for (int i = 0; i < numJoints - 1; i++) {
        float angle = convertPositionToAngle(joints[i], joints[i+1]);
        moveServo(i + 1, angle);
    }
}
float convertPositionToAngle(Point a, Point b) {
    // Placeholder calculation
    // In a real implementation, you'd consider the joint's axis of rotation
    // and potentially the previous joint's orientation.
    float deltaY = b.y - a.y;
    float deltaX = b.x - a.x;
    float angle = atan2(deltaY, deltaX) * 180 / PI; // Convert from radians to degrees
    return angle;
}




// Function to solve IK using FABRIK algorithm
void solveFABRIK(Point target) {
  Serial.println("Entered solveFABRIK function");
  Serial.print("Target X: "); Serial.println(target.x);
  Serial.print("Target Y: "); Serial.println(target.y);
  Serial.print("Target Z: "); Serial.println(target.z);
    Point base = joints[0]; // Remember the original base position
    int count = 0;

    float distance = distanceBetween(joints[3], target);
    while (distance > tolerance && count < maxIterations) {
        // Step 1: Forward reaching
        joints[4] = target; // Set the end-effector to the target
        for (int i = 3; i > 0; --i) {
            float r = distanceBetween(joints[i+1], joints[i]);
            float lambda = lengths[i] / r;
            joints[i] = {
                (1 - lambda) * joints[i+1].x + lambda * joints[i].x,
                (1 - lambda) * joints[i+1].y + lambda * joints[i].y,
                (1 - lambda) * joints[i+1].z + lambda * joints[i].z
            };
        }

        // Step 2: Backward reaching
        joints[0] = base; // Set back the base to its original position
        for (int i = 0; i < 3; ++i) {
            float r = distanceBetween(joints[i], joints[i+1]);
            float lambda = lengths[i] / r;
            joints[i+1] = {
                (1 - lambda) * joints[i].x + lambda * joints[i+1].x,
                (1 - lambda) * joints[i].y + lambda * joints[i+1].y,
                (1 - lambda) * joints[i].z + lambda * joints[i+1].z
            };
        }

        distance = distanceBetween(joints[3], target);
        count++;
    }

    // Apply the new joint positions to the robot
   // applyIKSolutionToRobot(joints);
}


//////////////////// Setup function///////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    bt1.begin(9600);

    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.begin(16, 4);      // set up the LCD's number of columns and rows
    lcd.setCursor(0, 0);
    lcd.print("2023 GDIP F1 :");
    lcd.setCursor(0, 1);
    lcd.print("6 DOF ROBOTIC ARM ");
    lcd.setCursor(0, 2);
    lcd.print("PROTOTYPE 4");

    // Initialize servo driver
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    joints[0] = {0, 0, 0}; // Base
    joints[1] = {0, 85, 0}; // First joint
    joints[2] = {0, 85 + 105, 0}; // Second joint
   
    // Initialize servoAngles array with predefined angles
    servoAngles[0] = 135;
    servoAngles[1] = 90;
    servoAngles[2] = 110;
    servoAngles[3] = 95;
    servoAngles[4] = 180;
    servoAngles[5] = 85;

    // Move servo motors to initial positions based on servoAngles
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
void loop() {
     String command = "";

    // Read commands from serial port or Bluetooth module
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received: " + command); // Echo the command back to the serial monitor
    } else if (bt1.available()) {
        command = bt1.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received via BT: " + command); // Echo the command from BT
    }

    // Execute command if available
    if (command != "") {
        executeCommand(command);
    }

}

//////////////////////////////////////////////////////////////////////////////////
void testFABRIK() {
    Point target = {100, 100, 100}; // Replace with your target position
    solveFABRIK(target);

    for (int i = 0; i < numJoints; i++) {
        Serial.print("Joint ");
        Serial.print(i);
        Serial.print(": (");
        Serial.print(joints[i].x);
        Serial.print(", ");
        Serial.print(joints[i].y);
        Serial.print(", ");
        Serial.print(joints[i].z);
        Serial.println(")");
    }
}
/////////////////// Function to start recording servo movements///////////////////////////
void StartRecordingMovements() {
    isRecord = true;
    indexRecord = 0;
    Serial.println("Recording started.");
    lcd.clear();
    lcd.print("Recording..."); // Show recording status on the LCD
}

/////////////////// Function to stop recording servo movements////////////////////////////////
void StopRecordingMovements() {
    isRecord = false;
    Serial.println("Recording stopped. Final recorded moves:");
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move "); Serial.print(i); Serial.println(":");
        for (int j = 0; j < servoNumber; j++) {
            Serial.print("Servo "); Serial.print(j + 1);
            Serial.print(": Angle "); Serial.println(movesServos[i][j]);
        }
    }
    lcd.clear();
    lcd.print("Recorded Moves:"); // Show how many moves were recorded
    lcd.setCursor(0, 1);
    lcd.print(indexRecord);
}

/////////////////////Function to play back recorded movements///////////////////////////
void PlayRecordedMovements() {
    Serial.println("Starting playback...");
    lcd.clear();
    lcd.print("Playing Moves..."); // Indicate playback on the LCD
    isPlay = true;
    for (int i = 0; i < indexRecord; i++) {  // Use < indexRecord to play up to the last recorded move
        Serial.print("Move: "); Serial.println(i);
        for (int j = 0; j < servoNumber; j++) {
            int pulse = pulseWidth(movesServos[i][j]);
            Serial.print("Servo "); Serial.print(j + 1); 
            Serial.print(" Angle: "); Serial.print(movesServos[i][j]);
            Serial.print(" Pulse Width: "); Serial.println(pulse);
            pwm.setPWM(j + 1, 0, pulse);
        }
        delay(1000);  // Delay between moves, adjust as needed
    }
    for (int i = 0; i < indexRecord; i++) {
        lcd.setCursor(0, 1);
        lcd.print("Move: ");
        lcd.print(i + 1); // Update the move number on the LCD
        // ... rest of the playback code
    }

    isPlay = false;
    Serial.println("Playback completed.");
    lcd.clear();
    lcd.print("Playback done."); // Indicate playback is done
}
//////////////////////////////////////////////////////////////////////////////////////////////////
// Function to print the pulse widths for the current angles
void printServoPulseWidths() {
    for (int i = 0; i < 6; i++) {
        int currentPulseWidth = pulseWidth(servoAngles[i]);
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(" Angle: ");
        Serial.print(servoAngles[i]);
        Serial.print(" Pulse Width: ");
        Serial.println(currentPulseWidth);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MoveToPick() {
    // Define the sequence of angles for each servo in the pick-up motion
    int angles[10][servoNumber] = {
        // HIP, WAIST, SHOULDER, ELBOW, WRIST, CLAW
        {125, 180, 190, 180, 190, 80},    // Initial position
        {150, 110, 100, 190, 90, 80}, // Move to above the object
        {150, 50, 180, 150, 180, 80}, // Lower towards the object
        {150, 50, 180, 150, 180, 140},// Close claw to grab the object
        {150, 90, 100, 150, 90, 140},// Lift the object
        {125, 180, 190, 180, 190, 120},    // Return to initial position with object
        {150, 110, 100, 190, 90, 80}, // Move the object
        {150, 40, 180, 150, 180, 80}, // Lower  the object
        {150, 40, 180, 150, 180, 80}, // OPEN claw to RELEASE the object
        {125, 180, 190, 180, 190, 120}    // Return to initial position with object
    };
        // Execute the sequence
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < servoNumber; j++) {
            pwm.setPWM(j + 1, 0, pulseWidth(angles[i][j]));
        }
        delay(2000); // Wait for 1 second between each step for smooth movement
    }

    // Optionally, open the claw to release the object at the end
    // pwm.setPWM(6, 0, pulseWidth(0)); // Open the claw
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void MoveToStart() {
    // Update the servoAngles array with the start positions
    // servoAngles[0] = 135;
    // servoAngles[1] = 190;
    // servoAngles[2] = 205;
    // servoAngles[3] = 200;
    // servoAngles[4] = 190;
    // servoAngles[5] = 85;
    servoAngles[0] = 135;
    servoAngles[1] = 90;
    servoAngles[2] = 110;
    servoAngles[3] = 95;
    servoAngles[4] = 180;
    servoAngles[5] = 85;

    // Use a loop to set each servo to its start position
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }

    Serial.println("Moved to start positions.");
    lcd.clear();
    lcd.print("Moved to Start"); // Indicate the arm has moved to the start position
    lcd.setCursor(0, 2);
    lcd.print("Warning ARM is ready");
}

    

//////////////////////////////////////////////////////////////////////////////////////////////////
// Correct the moveServo to use 0-based index for servoAngles array
void moveServo(int servoChannel, int angle) {
    int servoIndex = servoChannel - 1; // Convert 1-based index to 0-based index
    servoAngles[servoIndex] = angle;
    int pulse = pulseWidth(angle);

    Serial.print("Moving Servo "); Serial.print(servoChannel);
    Serial.print(" to Angle: "); Serial.print(angle);
    Serial.print(" (Pulse Width: "); Serial.print(pulse); Serial.println(")");

    pwm.setPWM(servoChannel, 0, pulse);
}


// Function to calculate pulse width for a given angle
int pulseWidth(int angle) {
    int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    return int(pulse_wide * FREQUENCY_SCALE);
}
//////////////////////////////////////////////////////////////////////////////////////////////

void executeCommand(String command) {
    command.trim(); // Trim whitespace
    command.toUpperCase(); // Convert to upper case for case-insensitive comparison

    // Define servo commands
    struct ServoCommand {
        const char* name;
        int channel;
    };

    ServoCommand commands[] = {
        {"HIP ", 1},
        {"WAIST ", 2},
        {"SHOULDER ", 3},
        {"ELBOW ", 4},
        {"WRIST ", 5},
        {"CLAW ", 6}
    };


    if (command.startsWith("SOLVE_FABRIK")) {
      Serial.println("Before SOLVE_FABRIK");
        // Parse the command to extract target coordinates
        // For simplicity, let's assume the command format is "SOLVE_FABRIK x,y,z"
        int firstCommaIndex = command.indexOf(',');
        int lastCommaIndex = command.lastIndexOf(',');
        
        String xStr = command.substring(14, firstCommaIndex); // Extract X coordinate
        String yStr = command.substring(firstCommaIndex + 1, lastCommaIndex); // Extract Y coordinate
        String zStr = command.substring(lastCommaIndex + 1); // Extract Z coordinate

        Point target = {
            xStr.toFloat(),
            yStr.toFloat(),
            zStr.toFloat()
        };
Serial.println("Before SOLVE_FABRIK");
        //solveFABRIK(target);
        
        // After solving, print the new joint positions
        for (int i = 0; i < numJoints; i++) {
            Serial.print("Joint ");
            Serial.print(i);
            Serial.print(": (");
            Serial.print(joints[i].x, 4); // Print x coordinate with 4 decimal places
            Serial.print(", ");
            Serial.print(joints[i].y, 4); // Print y coordinate with 4 decimal places
            Serial.print(", ");
            Serial.print(joints[i].z, 4); // Print z coordinate with 4 decimal places
            Serial.println(")");
        }
         return;

    }

    // Check for special commands first
    if (command == "MOVE_TO_PICK") {
        MoveToPick();
        Serial.println("Executing MoveToPick sequence");
    } else if (command == "GETVALUE") {
        printServoPulseWidths();
    } else if (command == "MOVE_TO_START") {
        MoveToStart();
        Serial.println("Executing MoveToStart");
    } else if (command == "START_RECORD") {
        StartRecordingMovements();
    } else if (command == "STOP_RECORD") {
        StopRecordingMovements();
    } else if (command == "PLAY_MOVEMENTS") {
        PlayRecordedMovements();
    } else if (command == "SAVE_MOVE") {  // New command to increment indexRecord
        if (isRecord && indexRecord < moveCount - 1) {
            indexRecord++;  // Increment the record index
            Serial.print("Move saved at index ");
            Serial.println(indexRecord);
        } else {
            Serial.println("Cannot save move, either not recording or out of space.");
        }
    } else {
        // Parse and execute servo commands
        bool isCommandExecuted = false;
        for (const auto& cmd : commands) {
            if (command.startsWith(cmd.name)) {
                isCommandExecuted = true;
                int angle = command.substring(strlen(cmd.name)).toInt();
                moveServo(cmd.channel, angle);
                Serial.println(String(cmd.name) + " moved to " + String(angle) + " degrees");

                // If we are in recording mode, save this movement
                if (isRecord && indexRecord < moveCount) {
                    movesServos[indexRecord][cmd.channel - 1] = angle;  // Store the angle in the recording array

                    Serial.print("Recording Move "); Serial.print(indexRecord);
                    Serial.print(" for Servo "); Serial.print(cmd.channel);
                    Serial.print(": Angle "); Serial.println(angle);
                }
                break; // Exit the loop after handling the command
            }
        }

        if (!isCommandExecuted) {
            // If the command was not recognized
            Serial.println("Unknown command: " + command);
        }
    }
}
