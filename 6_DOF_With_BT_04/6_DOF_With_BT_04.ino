//recording functions and testing
//LCD updates
//add are you sure feature and Initialize movesServos at the Start of Recording:
// speed control added and working on play back

// Arduino Robotic Arm Control with EEPROM Memory and Bluetooth Communication
// Features: Movement recording, playback, EEPROM storage, and Bluetooth command processing

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// Initialize components
LiquidCrystal_I2C lcd(0x27,16,4);  // LCD display setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Servo driver
SoftwareSerial bt1(2,3); // Bluetooth module (Rx, Tx)

// Servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50; // Servo frequency
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000; // Scale factor for pulse width

// Maximum number of moves and servos
const int moveCount = 20;
const int servoNumber = 6;
const int totalMoves = 20; 
const int bytesPerMove = 6; // EEPROM bytes per move

// Initial angles for each servo
int servoAngles[6] = {100, 90, 110, 95, 180, 0};

// Array to store servo movements
int movesServos[moveCount][servoNumber];

// Global state variables
bool waitingForConfirmation = false;
bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

int speed = 1; // Movement speed (delay)

// Function prototypes
int pulseWidth(int angle);
void executeCommand(String command);
void printServoPulseWidths();
void MoveToStart();
void moveServo(int servoChannel, int angle);
void StartRecordingMovements();
void StopRecordingMovements();
void PlayRecordedMovements();
void WriteMoveToEEPROM(int moveIndex, int servoAngles[]);
void DeleteRecordedMoves(int moveIndex);
void DeleteAllMoves();

// Setup function
void setup() {
    Serial.begin(9600); // Begin serial communication
    bt1.begin(9600); // Begin Bluetooth communication

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

    // Read the number of recorded movements from EEPROM
    indexRecord = EEPROM.read(0);
    if (EEPROM.read(0) == 255) { // Check if EEPROM is uninitialized
        EEPROM.write(0, 0); // Initialize indexRecord to 0
        DeleteAllMoves(); // Clear all servo positions in EEPROM
    } else {
        indexRecord = EEPROM.read(0);
    }

    // Move servo motors to initial positions
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }
}

// Main loop for processing commands and managing states
void loop() {
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
                bt1.println("Playback cancelled.");
            }
            waitingForConfirmation = false;
        } else {
            // Process incoming command
            executeCommand(command);
        }
    }
}

// Add more comments to remaining functions following the same pattern...
// Function to write recorded servo movements to EEPROM
void WriteMoveToEEPROM(int moveIndex, int servoAngles[]) {
    int startAddress = moveIndex * bytesPerMove; // Calculate starting address in EEPROM
    for (int i = 0; i < 6; i++) {
        EEPROM.write(startAddress + i, servoAngles[i]); // Write each angle to EEPROM
    }
}

// Function to delete a specific recorded move from EEPROM
void DeleteRecordedMoves(int moveIndex) {
    int startAddress = moveIndex * bytesPerMove; // Calculate starting address in EEPROM
    for (int i = 0; i < 6; i++) {
        EEPROM.write(startAddress + i, 0); // Reset angles to 0
    }
}

// Function to delete all recorded moves from EEPROM
void DeleteAllMoves() {
    for (int moveIndex = 0; moveIndex < totalMoves; moveIndex++) {
        int startAddress = moveIndex * bytesPerMove; // Calculate starting address in EEPROM
        for (int i = 0; i < 6; i++) {
            EEPROM.write(startAddress + i, 0); // Reset angles to 0 for each move
        }
    }
}

// Function to start recording servo movements
void StartRecordingMovements() {
    isRecord = true; // Set recording state to true
    indexRecord = 0; // Reset index of recorded moves

    // Initialize movesServos with current servo angles
    for (int i = 0; i < servoNumber; i++) {
        for (int j = 0; j < moveCount; j++) {
            movesServos[j][i] = servoAngles[i];
        }
    }

    // Notify user that recording has started
    Serial.println("Recording started.");
    lcd.clear();
    lcd.print("Recording...");
    bt1.println("Recording started.");
}

// Function to stop recording servo movements and save them to EEPROM
void StopRecordingMovements() {
    isRecord = false; // Set recording state to false
    EEPROM.write(0, indexRecord); // Save the number of recorded moves to EEPROM

    // Store each move's servo angles in EEPROM
    for (int i = 0; i < indexRecord; i++) {
        for (int j = 0; j < servoNumber; j++) {
            int eepromAddress = 1 + i * servoNumber + j; // Calculate EEPROM address
            EEPROM.write(eepromAddress, movesServos[i][j]); // Write angle to EEPROM
        }
    }

    // Notify user that recording has stopped
    Serial.println("Recording stopped. Final recorded moves:");
    bt1.println("Recording stopped. Final recorded moves:");
    lcd.clear();
    lcd.print("Recorded Moves:");
    lcd.setCursor(0, 1);
    lcd.print(indexRecord);
}

// Function to play back recorded movements from EEPROM
void PlayRecordedMovements() {
    // Notify user that playback is starting
    Serial.println("Starting playback...");
    bt1.println("Starting playback...");
    lcd.clear();
    lcd.print("Playing Moves...");
    isPlay = true; // Set playback state to true

    // Iterate through each recorded move
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move: "); Serial.println(i);
        bt1.print("Move: "); bt1.println(i);
        for (int j = 0; j < servoNumber; j++) {
            int eepromAddress = 1 + i * servoNumber + j; // Calculate EEPROM address
            int angle = EEPROM.read(eepromAddress); // Read angle from EEPROM
            int pulse = pulseWidth(angle); // Calculate pulse width
            Serial.print("Servo "); Serial.print(j + 1);
            Serial.print(" Angle: "); Serial.print(angle);
            Serial.print(" Pulse Width: "); Serial.println(pulse);
            bt1.print("Servo "); bt1.print(j + 1);
            bt1.print(" Angle: "); bt1.print(angle);
            bt1.print(" Pulse Width: "); bt1.println(pulse);
            moveServo(j + 1, angle); // Move servo to recorded angle
        }
        delay(1000); // Delay between moves
    }

    // Notify user that playback is completed
    Serial.println("Playback completed.");
    bt1.println("Playback completed.");
    isPlay = false; // Set playback state to false
    for (int i = 0; i < indexRecord; i++) {
        lcd.setCursor(0, 1);
        lcd.print("Move: ");
        lcd.print(i + 1);
    }
}

// Additional function comments to follow the same pattern...
// Function to print the pulse widths for the current servo angles
void printServoPulseWidths() {
    for (int i = 0; i < 6; i++) {
        int currentPulseWidth = pulseWidth(servoAngles[i]); // Calculate pulse width
        // Print angle and pulse width to Serial and Bluetooth
        Serial.print("Servo "); Serial.print(i); Serial.print(" Angle: ");
        Serial.print(servoAngles[i]); Serial.print(" Pulse Width: ");
        Serial.println(currentPulseWidth);
        bt1.print("Servo "); bt1.print(i); bt1.print(" Angle: ");
        bt1.print(servoAngles[i]); bt1.print(" Pulse Width: ");
        bt1.println(currentPulseWidth);
    }
}

// Function to move the robotic arm to its starting position
void MoveToStart() {
    // Move each servo to its starting position
    for (int i = 0; i < 6; i++) {
        moveServo(i + 1, servoAngles[i]);
    }
    // Update servoAngles array to reflect starting positions
    servoAngles[0] = 100; // HIP servo
    servoAngles[1] = 90;  // WAIST servo
    servoAngles[2] = 10;  // SHOULDER servo
    servoAngles[3] = 95;  // ELBOW servo
    servoAngles[4] = 180; // WRIST servo
    servoAngles[5] = 0;   // CLAW servo

    // Notify user that the arm has moved to the start positions
    Serial.println("Moved to start positions.");
    bt1.println("Moved to start positions.");
    lcd.clear();
    lcd.print("Moved to Start");
    lcd.setCursor(0, 2);
    lcd.print("Warning ARM is ready");
}   

// Function to move a specific servo to a target angle
void moveServo(int servoChannel, int targetAngle) {
    if (targetAngle < 0 || targetAngle > 180) {
        // Handle invalid angle input
        Serial.println("Error: Target angle out of range (0-180 degrees).");
        return; // Exit the function without moving the servo
    }

    // Convert 1-based servo channel index to 0-based array index
    int servoIndex = servoChannel - 1; 
    
    // Get current angle and determine movement direction
    int currentAngle = servoAngles[servoIndex];
    int stepSize = 8; // Define step size for gradual movement
    int step = (currentAngle < targetAngle) ? stepSize : -stepSize;

    // Gradually move the servo to the target angle
    while (true) {
        if ((step > 0 && currentAngle + step > targetAngle) || 
            (step < 0 && currentAngle + step < targetAngle)) {
            currentAngle = targetAngle; // Prevent overshoot
        } else {
            currentAngle += step; // Increment angle
        }

        // Update servoAngles array and calculate pulse width
        servoAngles[servoIndex] = currentAngle;
        int pulse = pulseWidth(currentAngle);

        // Output movement information and set servo position
        Serial.print("Moving Servo "); Serial.print(servoChannel);
        Serial.print(" to Angle: "); Serial.print(currentAngle);
        Serial.print(" (Pulse Width: "); Serial.print(pulse); Serial.println(")");
        bt1.print("Moving Servo "); bt1.print(servoChannel);
        bt1.print(" to Angle: "); bt1.print(currentAngle);
        bt1.print(" (Pulse Width: "); bt1.print(pulse); bt1.println(")");
        pwm.setPWM(servoChannel, 0, pulse);

        delay(speed); // Delay for controlled movement

        if (currentAngle == targetAngle) {
            break; // Stop moving when target angle is reached
        }
    }
}

// Function to calculate pulse width based on a given servo angle
int pulseWidth(int angle) {
    int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); // Map angle to pulse width
    return int(pulse_wide * FREQUENCY_SCALE); // Calculate actual pulse width
}

// Function to execute commands received from Serial or Bluetooth
void executeCommand(String command) {
    command.trim(); // Remove leading/trailing whitespace
    command.toUpperCase(); // Convert to uppercase for case-insensitive comparison

    // Array of known servo commands
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

    // Handling various commands
    if (command.startsWith("DELETE_MOVE")) {
        // Delete specific move from EEPROM
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex != -1) {
            int moveIndex = command.substring(spaceIndex + 1).toInt();
            DeleteRecordedMoves(moveIndex);
            Serial.println("Move " + String(moveIndex) + " deleted.");
        } else {
            Serial.println("Invalid command format.");
        }
    } else if (command == "DELETE_ALL_MOVES") {
        // Delete all moves from EEPROM
        DeleteAllMoves();
        Serial.println("All moves deleted.");
    } else if (command == "PLAY_MOVEMENTS") {
        // Request confirmation to play movements
        waitingForConfirmation = true;
        Serial.println("Are you sure you want to play movements? (YES/NO)");
        bt1.println("Are you sure you want to play movements? (YES/NO)");
    } else if (command.startsWith("SPEED ")) {
        // Adjust movement speed
        String speedValueString = command.substring(6); // Extract the speed value
        int newSpeed = speedValueString.toInt(); // Convert to integer
        if (newSpeed > 0) {
            speed = newSpeed; // Update global speed variable
            Serial.print("Speed updated to: ");
            Serial.println(speed);
        } else {
            Serial.println("Invalid speed value"); // Handle invalid input
        }
    }
    else if (command == "GETVALUE") {
        // Print current servo pulse widths
        printServoPulseWidths();
    }
    else if (command == "MOVE_TO_START") {
        // Move the robotic arm to its starting position
        MoveToStart();
        Serial.println("Executing MoveToStart");
    }
    else if (command == "START_RECORD") {
        // Start recording servo movements
        StartRecordingMovements();
    }
    else if (command == "STOP_RECORD") {
        // Stop recording servo movements and save to EEPROM
        StopRecordingMovements();
    }
    else if (command == "PLAY_MOVEMENTS") {
        // Play back recorded movements from EEPROM
        PlayRecordedMovements();
    }
    else if (command == "SAVE_MOVE") {
        // Save the current servo positions as a move
        if (isRecord && indexRecord < moveCount - 1) {
            indexRecord++;  // Increment the record index
            Serial.print("Move saved at index ");
            Serial.println(indexRecord);
            bt1.print("Move saved at index ");
            bt1.println(indexRecord);
        } else {
            Serial.println("Cannot save move, either not recording or out of space.");
            bt1.println("Cannot save move, either not recording or out of space.");
        }
    } else {
        // Handle servo-specific commands
        bool isCommandExecuted = false;
        for (const auto& cmd : commands) {
            if (command.startsWith(cmd.name)) {
                isCommandExecuted = true;
                int angle = command.substring(strlen(cmd.name)).toInt();
                moveServo(cmd.channel, angle); // Move specified servo
                Serial.println(String(cmd.name) + " moved to " + String(angle) + " degrees");

                // Save this movement if in recording mode
                if (isRecord && indexRecord < moveCount) {
                    movesServos[indexRecord][cmd.channel - 1] = angle;  // Store the angle
                    Serial.print("Recording Move "); Serial.print(indexRecord);
                    Serial.print(" for Servo "); Serial.print(cmd.channel);
                    Serial.print(": Angle "); Serial.println(angle);
                }
                break; // Command processed, exit loop
            }
        }

        if (!isCommandExecuted) {
            // Notify user if command was not recognized
            Serial.println("Unknown command: " + command);
            bt1.println("Unknown command: " + command);
        }
    }
}


