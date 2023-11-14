//recording functions and testing
//LCD updates


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
// Define maximum moves and servos
const int moveCount = 10;
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
        bt1.println("Received: " + command); // Added line
    } else if (bt1.available()) {
        command = bt1.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received via BT: " + command); // Echo the command from BT
        bt1.println("Received via BT: " + command); // Echo the command from BT
    }

    // Execute command if available
    if (command != "") {
        executeCommand(command);
    }

}

/////////////////// Function to start recording servo movements///////////////////////////
void StartRecordingMovements() {
    isRecord = true;
    indexRecord = 0;
    Serial.println("Recording started.");
    lcd.clear();
    lcd.print("Recording..."); // Show recording status on the LCD
    bt1.println("Recording started."); // Adding Bluetooth print
}

/////////////////// Function to stop recording servo movements////////////////////////////////
void StopRecordingMovements() {
    isRecord = false;
    Serial.println("Recording stopped. Final recorded moves:");
    bt1.println("Recording stopped. Final recorded moves:");  // Added
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move "); Serial.print(i); Serial.println(":");
        bt1.print("Move "); bt1.print(i); bt1.println(":");  // Added
        for (int j = 0; j < servoNumber; j++) {
            Serial.print("Servo "); Serial.print(j + 1);
            Serial.print(": Angle "); Serial.println(movesServos[i][j]);
            bt1.print("Servo "); bt1.print(j + 1);  // Added
            bt1.print(": Angle "); bt1.println(movesServos[i][j]);  // Added
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
    bt1.println("Starting playback...");  // Added
    lcd.clear();
    lcd.print("Playing Moves..."); // Indicate playback on the LCD
    isPlay = true;
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move: "); Serial.println(i);
        bt1.print("Move: "); bt1.println(i);  // Added
        for (int j = 0; j < servoNumber; j++) {
            int pulse = pulseWidth(movesServos[i][j]);
            Serial.print("Servo "); Serial.print(j + 1); 
            Serial.print(" Angle: "); Serial.print(movesServos[i][j]);
            Serial.print(" Pulse Width: "); Serial.println(pulse);
            bt1.print("Servo "); bt1.print(j + 1);  // Added
            bt1.print(" Angle: "); bt1.print(movesServos[i][j]);  // Added
            bt1.print(" Pulse Width: "); bt1.println(pulse);  // Added
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
    bt1.println("Playback completed.");  // Added
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
        bt1.print("Servo ");  // Added
        bt1.print(i);  // Added
        bt1.print(" Angle: ");  // Added
        bt1.print(servoAngles[i]);  // Added
        bt1.print(" Pulse Width: ");  // Added
        bt1.println(currentPulseWidth);  // Added
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
    bt1.println("Moved to start positions.");  // Added
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
    bt1.print("Moving Servo "); bt1.print(servoChannel);
    bt1.print(" to Angle: "); bt1.print(angle);
    bt1.print(" (Pulse Width: "); bt1.print(pulse); bt1.println(")"); // Adding Bluetooth print

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
