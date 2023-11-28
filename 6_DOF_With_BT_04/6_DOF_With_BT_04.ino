//recording functions and testing
//LCD updates
//add are you sure feature and Initialize movesServos at the Start of Recording:
// speed control added and working on play back


#include <EEPROM.h>
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
//SoftwareSerial bt1(7,6); /* (Rx,Tx)  for Jakes PCB*/

// Define servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000;
// Define maximum moves and servos
 const int moveCount = 20;
 const int servoNumber = 6;
const int totalMoves = 20;  // Adjust as needed
const int bytesPerMove = 6;  // Assuming each move takes up 6 bytes
// Define the LED pins
const int greenLEDPin = 3;   // Green LED on digital pin D3
const int yellowLEDPin = 4;  // Yellow LED on digital pin D4
const int redLEDPin = 5;     // Red LED on digital pin D5

int servoAngles[6] = {100, 90, 110, 95, 180, 0}; // Initial angles for each servo
int movesServos[moveCount][servoNumber]; //max of 10 moves
// Global variable to track state
bool waitingForConfirmation = false; // safety check, stop user from acidently runing the squence when not ready
bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

int speed = 1; // set the speed (delay) of movment

// Function Prototypes function must be declared before it is called unless it is defined above the point where it is called.
int pulseWidth(int angle);
void executeCommand(String command);
void printServoPulseWidths();
//void MoveToPick();
void MoveToStart();
void moveServo(int servoChannel, int angle);
void StartRecordingMovements();
void StopRecordingMovements();
void PlayRecordedMovements();
void WriteMoveToEEPROM(int moveIndex, int servoAngles[]);
void DeleteRecordedMoves(int moveIndex);
void DeleteAllMoves();

//////////////////// Setup function///////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    bt1.begin(9600);

     // Initialize the LED pins as outputs
     pinMode(greenLEDPin, OUTPUT);
     pinMode(yellowLEDPin, OUTPUT);
     pinMode(redLEDPin, OUTPUT);

    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("2023 GDIP F1 :");
    lcd.setCursor(0, 1);
    lcd.print("6 DOF ROBOTIC ARM ");
    lcd.setCursor(0, 2);
    lcd.print("PROTOTYPE 4");

    // Initialize servo driver
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    indexRecord = EEPROM.read(0); // Read the stored indexRecord from EEPROM
    // Check if EEPROM is initialized
    if (EEPROM.read(0) == 255) { // 255 can indicate uninitialized EEPROM
        EEPROM.write(0, 0); // Initialize indexRecord to 0
        DeleteAllMoves(); // Clear all servo positions in EEPROM
    } else {
        indexRecord = EEPROM.read(0);
    }

   
    // // Initialize servoAngles array with predefined angles
    // servoAngles[0] = 100;
    // servoAngles[1] = 90;
    // servoAngles[2] = 110;
    // servoAngles[3] = 95;
    // servoAngles[4] = 180;
    // servoAngles[5] = 0;

    // Move servo motors to initial positions based on servoAngles
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
void loop() {

  if (isRecord) {
    // Turn the yellow LED on and others off
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(yellowLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
  } else {
    // Turn the Green LED on and others off
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(yellowLEDPin, LOW);
    digitalWrite(redLEDPin, LOW);
  }
  if (isplay) {
    // Turn the RED LED on and others off
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(yellowLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
  } 



    String command = "";

    if (Serial.available() || bt1.available()) {
        // Read command from Serial or Bluetooth
        command = (Serial.available() ? Serial.readStringUntil('\n') : bt1.readStringUntil('\n'));
        command.trim();

        if (waitingForConfirmation) {
            if (command.equalsIgnoreCase("YES")) {
                PlayRecordedMovements();
            } else {
                Serial.println("Playback cancelled.");
                bt1.println("Playback cancelled."); // Send cancellation message over Bluetooth
            }
            waitingForConfirmation = false; // Reset the confirmation flag
        } else {
            // Process the command normally
            executeCommand(command);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////
void WriteMoveToEEPROM(int moveIndex, int servoAngles[]) {
    int startAddress = moveIndex * bytesPerMove;
    for (int i = 0; i < 6; i++) {
        EEPROM.write(startAddress + i, servoAngles[i]);
    }
}
///////////////////////////////////////////////////////////////////////////////////////
void DeleteRecordedMoves(int moveIndex) {
    int startAddress = moveIndex * bytesPerMove;
    for (int i = 0; i < 6; i++) {
        EEPROM.write(startAddress + i, 0);
    }
}
/////////////////////////////////////////////////////////////////////////////////////
void DeleteAllMoves() {
    for (int moveIndex = 0; moveIndex < totalMoves; moveIndex++) {
        int startAddress = moveIndex * bytesPerMove;
        for (int i = 0; i < 6; i++) {
            EEPROM.write(startAddress + i, 0);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////

/////////////////// Function to start recording servo movements///////////////////////////
void StartRecordingMovements() {
    isRecord = true;
    indexRecord = 0;
    

    // Initialize movesServos with current servo angles
    for (int i = 0; i < servoNumber; i++) {
        for (int j = 0; j < moveCount; j++) {
            movesServos[j][i] = servoAngles[i];
        }
    }

    Serial.println("Recording started.");
    lcd.clear();
    lcd.print("Recording...");
    bt1.println("Recording started.");
}



/////////////////// Function to stop recording servo movements////////////////////////////////
void StopRecordingMovements() {
    
    Serial.println("Recording stopped. Final recorded moves:");
    bt1.println("Recording stopped. Final recorded moves:");

     isRecord = false;
    EEPROM.write(0, indexRecord); // Store indexRecord at the beginning of EEPROM

    // Other code for storing servo movements in EEPROM
    // Adjust address calculation to account for the first byte being used
    for (int i = 0; i < indexRecord; i++) {
        for (int j = 0; j < servoNumber; j++) {
            int eepromAddress = 1 + i * servoNumber + j; // Start from address 1
            EEPROM.write(eepromAddress, movesServos[i][j]);
        }
    }
    lcd.clear();
    lcd.print("Recorded Moves:");
    lcd.setCursor(0, 1);
    lcd.print(indexRecord);
}


/////////////////////Function to play back recorded movements///////////////////////////
void PlayRecordedMovements() {
    Serial.println("Starting playback...");
    bt1.println("Starting playback...");
    lcd.clear();
    lcd.print("Playing Moves...");
    isPlay = true;

    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move: "); Serial.println(i);
        bt1.print("Move: "); bt1.println(i);
        for (int j = 0; j < servoNumber; j++) {
            int eepromAddress = 1 + i * servoNumber + j; // Adjusted address
            int angle = EEPROM.read(eepromAddress);
            int pulse = pulseWidth(angle);
            Serial.print("Servo "); Serial.print(j + 1);
            Serial.print(" Angle: "); Serial.print(angle);
            Serial.print(" Pulse Width: "); Serial.println(pulse);
            bt1.print("Servo "); bt1.print(j + 1);
            bt1.print(" Angle: "); bt1.print(angle);
            bt1.print(" Pulse Width: "); bt1.println(pulse);
            moveServo(j + 1, angle);
        }
        delay(1000); // Delay between moves, adjust as needed
    }

    for (int i = 0; i < indexRecord; i++) {
        lcd.setCursor(0, 1);
        lcd.print("Move: ");
        lcd.print(i + 1);
    }

    isPlay = false;
    Serial.println("Playback completed.");
    bt1.println("Playback completed.");

    // Turn the RED LED on and others off
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(yellowLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
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
        bt1.print("Servo ");  // Added
        bt1.print(i);  // Added
        bt1.print(" Angle: ");  // Added
        bt1.print(servoAngles[i]);  // Added
        bt1.print(" Pulse Width: ");  // Added
        bt1.println(currentPulseWidth);  // Added
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// void MoveToPick() {
//     // Define the sequence of angles for each servo in the pick-up motion
//     int angles[10][servoNumber] = {
//         // HIP, WAIST, SHOULDER, ELBOW, WRIST, CLAW
//         {125, 180, 190, 180, 190, 0},    // Initial position
//         {150, 110, 100, 190, 90, 0}, // Move to above the object
//         {150, 50, 180, 150, 180, 0}, // Lower towards the object
//         {150, 50, 180, 150, 180, 0},// Close claw to grab the object
//         {150, 90, 100, 150, 90, 0},// Lift the object
//         {125, 180, 190, 180, 190, 0},    // Return to initial position with object
//         {150, 110, 100, 190, 90, 0}, // Move the object
//         {150, 40, 180, 150, 180, 0}, // Lower  the object
//         {150, 40, 180, 150, 180, 0}, // OPEN claw to RELEASE the object
//         {125, 180, 190, 180, 190, 0}    // Return to initial position with object
//     };
//         // Execute the sequence
//     for (int i = 0; i < 10; i++) {
//         for (int j = 0; j < servoNumber; j++) {
//             pwm.setPWM(j + 1, 0, pulseWidth(angles[i][j]));
//         }
//         delay(2000); // Wait for 1 second between each step for smooth movement
//     }

//     // Optionally, open the claw to release the object at the end
//     // pwm.setPWM(6, 0, pulseWidth(0)); // Open the claw
// }
/////////////////////////////////////////////////////////////////////////////////////////////////
void MoveToStart() {


     for (int i = 0; i < 6; i++) {
        moveServo(i + 1, servoAngles[i]);
    }
    servoAngles[0] = 100; //HIP servo
    servoAngles[1] = 90;  //WAIST servo
    servoAngles[2] = 10;  //SHOUDLER servo
    servoAngles[3] = 95;  //ELBOW servo
    servoAngles[4] = 180; //WRIST servo
    servoAngles[5] = 0;   //CLAW servo

    // Use a loop to set each servo to its start position using moveServo
   
    Serial.println("Moved to start positions.");
    bt1.println("Moved to start positions.");  // Added
    lcd.clear();
    lcd.print("Moved to Start"); // Indicate the arm has moved to the start position
    lcd.setCursor(0, 2);
    lcd.print("Warning ARM is ready");
}   
//////////////////////////////////////////////////////////////////////////////////////////////////
//Correct the moveServo to use 0-based index for servoAngles array
void moveServo(int servoChannel, int targetAngle) {
        if (targetAngle < 0 || targetAngle > 180) {
        Serial.println("Error: Target angle out of range (0-180 degrees).");
        return; // Exit the function without moving the servo
    }

    int servoIndex = servoChannel - 1; // Convert 1-based index to 0-based index
    
    // Get current angle
    int currentAngle = servoAngles[servoIndex];

    int stepSize = 8; // Define your step size here
    int step = (currentAngle < targetAngle) ? stepSize : -stepSize;

    // Gradually move the servo from its current angle to the target angle
    while (true) {
        // Check if the next step will overshoot the target
        if ((step > 0 && currentAngle + step > targetAngle) || 
            (step < 0 && currentAngle + step < targetAngle)) {
            currentAngle = targetAngle; // Set to target angle to prevent overshoot
        } else {
            currentAngle += step; // Move to the next step
        }

        servoAngles[servoIndex] = currentAngle;
        int pulse = pulseWidth(currentAngle);

        // Debug information
        Serial.print("Moving Servo "); Serial.print(servoChannel);
        Serial.print(" to Angle: "); Serial.print(currentAngle);
        Serial.print(" (Pulse Width: "); Serial.print(pulse); Serial.println(")");

        // Bluetooth information
        bt1.print("Moving Servo "); bt1.print(servoChannel);
        bt1.print(" to Angle: "); bt1.print(currentAngle);
        bt1.print(" (Pulse Width: "); bt1.print(pulse); bt1.println(")");

        // Set the servo position
        pwm.setPWM(servoChannel, 0, pulse);

        // Delay to control the speed
        delay(speed);

        // Break out of the loop once the target angle is reached
        if (currentAngle == targetAngle) {
            break;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
// Function to calculate pulse width for a given angle remains the same
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

    // Check for special commands first
    // if (command == "MOVE_TO_PICK") {
    //     MoveToPick();
    //     Serial.println("Executing MoveToPick sequence");
    // } else 
      if (command.startsWith("DELETE_MOVE")) {
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex != -1) {
            int moveIndex = command.substring(spaceIndex + 1).toInt();
            DeleteRecordedMoves(moveIndex);
            Serial.println("Move " + String(moveIndex) + " deleted.");
        } else {
            Serial.println("Invalid command format.");
        }
    } else if (command == "DELETE_ALL_MOVES") {
        DeleteAllMoves();
        Serial.println("All moves deleted.");
    } else if (command == "PLAY_MOVEMENTS") {
        waitingForConfirmation = true;
        Serial.println("Are you sure you want to play movements? (YES/NO)");
        bt1.println("Are you sure you want to play movements? (YES/NO)"); // Send confirmation request over Bluetooth
    }
    else if (command.startsWith("SPEED ")) {
        String speedValueString = command.substring(6); // Extract the part of the string after "SPEED "
        int newSpeed = speedValueString.toInt(); // Convert the extracted part to an integer
        if(newSpeed > 0) { // Optionally check if the new speed is a positive number
            speed = newSpeed; // Update the global speed variable

            // Optional: Acknowledge the speed change
            Serial.print("Speed updated to: ");
            Serial.println(speed);
        } else {
            // Handle invalid speed value
            Serial.println("Invalid speed value");
        }
    }
    else if (command == "GETVALUE") {
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
            bt1.print("Move saved at index "); // Adding Bluetooth print
            bt1.println(indexRecord); // Adding Bluetooth print
        } else {
            Serial.println("Cannot save move, either not recording or out of space.");
            bt1.println("Cannot save move, either not recording or out of space.");
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
            bt1.println("Unknown command: " + command); // Adding Bluetooth print
        }
    }
}
