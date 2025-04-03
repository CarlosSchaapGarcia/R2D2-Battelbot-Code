// Gripper Pin
const int servoPin = 12;
// Motor Pins
const int motor_A2 = 10; // Left forward 
const int motor_A1 = 11; // Left backward 
const int motor_B2 = 8;  // Right backward 
const int motor_B1 = 9;  // Right forward 

bool gripperOpen = false;
unsigned long lastActionTime = 0;
const unsigned long actionInterval = 1000; // 1 second between actions
int stepCount = 0; // Track how many movements have occurred

unsigned long gripperCloseTime = 0; // Store time when gripper was closed
const unsigned long gripperHoldInterval = 200; // 200 ms to ensure servo holds position

bool gripperLocked = false; // Lock the gripper to prevent accidental opening

void setup() 
{
    pinMode(servoPin, OUTPUT);
    moveServo(0); // Start with gripper closed
    gripperCloseTime = millis(); // Record time when gripper was initially closed
}

void loop() {
    if (stepCount < 3 && millis() - lastActionTime > actionInterval) 
    { 
        if (stepCount == 0 && !gripperLocked) moveServo(180); // Open first time
        else if (stepCount == 1 && !gripperLocked) moveServo(0); // Close
        else if (stepCount == 2 && !gripperLocked) moveServo(180); // Open again

        stepCount++; // Move to the next step
        lastActionTime = millis(); // Update last action time
    }
    else if (stepCount == 3) 
    {  
        moveToCone(); // Move the robot towards the detected object
        stepCount++;  // Prevent re-running
    }
    maintainGripperClosed(); // Ensure the gripper stays closed during step 3
}

// Function to move servo without a library
void moveServo(int angle) 
{
    int pulseWidth = map(angle, 0, 180, 1000, 2000); 

    // Generate 50Hz signal for ~1 second to move the servo
    for (int i = 0; i < 50; i++) { 
        digitalWrite(servoPin, HIGH);
        delayMicroseconds(pulseWidth); 
        digitalWrite(servoPin, LOW);
        delayMicroseconds(20000 - pulseWidth); 
    }
}

void moveToCone()
{
        moveForward(800); // Move towards the cone
        stopMotors(500);  // Stop and wait
        lockGripperClosed(); // Ensure the gripper stays closed during this step
        moveForward(800); // Move towards the cone again
        stopMotors(2000); // Stop and hold
}

// Move forward
void moveForward(int delayTime) 
{
    analogWrite(motor_A1, 0);
    analogWrite(motor_A2, 255);
    analogWrite(motor_B1, 255);
    analogWrite(motor_B2, 0);
    delay(delayTime);
}

// Move backward
void moveBackward() 
{
    analogWrite(motor_A1, 255);
    analogWrite(motor_A2, 0);
    analogWrite(motor_B1, 0);
    analogWrite(motor_B2, 255);
}

// Stop
void stopMotors(int delayTime) 
{
    analogWrite(motor_A1, 0);
    analogWrite(motor_A2, 0);
    analogWrite(motor_B1, 0);
    analogWrite(motor_B2, 0);
    delay(delayTime);
}

// Lock the gripper in the closed position
void lockGripperClosed() 
{
    moveServo(0);  // Ensure the servo stays at the closed position
    gripperLocked = true;  // Lock the gripper to prevent future opening
    gripperCloseTime = millis(); // Store the time when the gripper was closed
}

// Maintain the gripper in the closed position during Step 3
void maintainGripperClosed() 
{
    if (gripperLocked && millis() - gripperCloseTime > gripperHoldInterval) 
    {
        // If enough time has passed since the gripper was closed, continue sending the pulse to keep it closed
        moveServo(0); // Keep the gripper in the closed position
        gripperCloseTime = millis(); // Reset the close time
    }
}

// Function to check distance
int getDistance() 
{
    long duration;
    int distance;

    // Ensure the trigger pin is LOW before sending a pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);  // Wait for the sensor to be ready

    // Send a 10-microsecond HIGH pulse to trigger the sensor
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure the time taken for the echo to return
    duration = pulseIn(ECHO_PIN, HIGH);

    // Convert the duration to distance (in cm)
    distance = duration * 0.034 / 2;

    return distance;  // Return the measured distance in cm
}
