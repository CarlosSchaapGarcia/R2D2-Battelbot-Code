// LIBRARY AND PINS FOR NEO PIXELS
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN  6 // Pin connecting NEO pixel Leds
#define NUM_PIXELS    4 // Number of NEO pixels

// PINS CONNECTTING THE ULTRA SENSORS
#define ECHO_PIN      2 // Pin connecting the ultra sensors
#define TRIG_PIN      3 // Pin connecting the ultra sensors

// PINS FOR THE GRIPPER
#define SERVO_PIN     12 //
#define GRIPPER_PIN   12 //

// PINS FOR THE GRIPPER MOTORS
#define MOTOR_A2      10 // Left forward 
#define MOTOR_A1      11 // Left backward 
#define MOTOR_B2      8  // Right backward 
#define MOTOR_B1      9  // Right forward 


// UNSIGNED LOGS
unsigned long raceTimer = 0;        // Used for timing the race
unsigned long previousMillis = 0;   // Stores the last time a T-junction was detected
const unsigned long delayTime = 75; // 0.5 seconds in milliseconds
const unsigned long actionInterval[] = {500, 400, 700, 400, 350, 400, 700, 350, 400, 700}; // Array for the obstacle avoidance



// BOOLEANS
bool isAtTjunction = false;     // Boolean for T junction
bool isRaceFinished = false;    // Boolean to check if the race is finished, if it is it will run the celebration code
bool isRaceExit = false;        // Boolean for when the celebration is done we stop the motors.
bool avoidingObstacle = false;  // Boolean to check if it is running the avoid obstacle 
bool startUpCompleted = false;  // Boolean to run the startup
bool calibrated = false;        // Boolean to check if the calibration code is already done


// PINS FOR THE ULTRASONIC SENSORS
int distance = 0;

// PID SETTINGS
const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int _sensorValues[8];
int _sensorThreshold = 700;
int _sensorWeights[] =  { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

float _Kp = 0.065;    // Proportional gain (controls how much the robot adjusts based on the error)
float _Ki = 0.0045;   // Integral gain (helps correct small, long-term errors)
float _Kd = 0.0085;   // Derivative gain (helps dampen sudden changes in error)

int _previousError = 0;
int _P = 0;             // Store the previous error value for calculating the derivative term
int _integral = 0;      // Integral of the error, to help fix long-term errors
int _baseSpeed = 190;

// NEO PIXELS SETTINGS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SETUP
void setup() {

  pinMode(MOTOR_A1, OUTPUT); 
  pinMode(MOTOR_A2, OUTPUT); 
  pinMode(MOTOR_B1, OUTPUT); 
  pinMode(MOTOR_B2, OUTPUT);
    
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GRIPPER_PIN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  strip.begin();
  strip.show(); 
}   

void loop() {
  if(!startUpCompleted) {
    startUp();
  }
  raceTimer = millis();
  if(millis() % raceTimer < 3 ) {
  distance = getDistance();
  }
  if(distance > 15) {
    int newDistance = getDistance();
    if(newDistance > 15) {
      if(!calibrated) {
        int _sensorThreshold = calibrate();  
        calibrated = true;   
      }
    }
  }
  if(calibrated) {
    if(!isRaceFinished) {
      if(!avoidingObstacle && distance > 15) {
        int position = readSensors();
        int error = position;
        PIDcontrol(error);
      }
      else if(distance < 13) {
        avoidingObstacle = true;
        obstacleAvoid();
      }
    } 
    else {
      if(!isRaceExit)
      exitRace();
    }
  }
}

// CODE FOR READING THE LINE FOLLOWER SENSORS
int readSensors() {
  int weightedSum = 0;  // Sum of all sensor weights
  int sum = 0;          // Number of active sensors
  bool allBlack = true; // Flag to check if all sensors are black

  for (int i = 0; i < 8; i++) {
    _sensorValues[i] = analogRead(sensorPins[i]);
    if (_sensorValues[i] > _sensorThreshold) {
      weightedSum += _sensorWeights[i];
      sum++;
      allBlack = false; // At least one sensor is not black
    }
  }
  // If all sensors are black (below threshold), return 9999
  if (allBlack) {
    return 0;
  }
  if (sum == 0) {
    return (_previousError > 0) ? 1000 : -1000;
  }
  return weightedSum / sum;
}

// PID CONTROL CODE
void PIDcontrol(int error) {
  // Check if we're at a T-junction (both error and previousError are 0)
  if (error == 0 && _previousError == 0) {
    // If it's the first time detecting the T-junction, start timing
    if (!isAtTjunction) {
      previousMillis = millis();  // Record the time of the first detection
      isAtTjunction = true;  // Set the flag indicating we're at a T-junction
    }
    // Check if 0.5 seconds have passed since the T-junction was detected
    if (millis() - previousMillis >= delayTime) {    
      isRaceFinished = true;
      _previousError = 0; // Reset previousError when stopping at a T-junction
      return; // Exit the PID control function since the robot is at a T-junction for long enough
      }
    } else {
      isAtTjunction = false; // If we're no longer at the T-junction, reset the flag
    }

  // Regular PID control calculations
  _P = error;
  _integral += error;
  _integral = constrain(_integral, -1000, 1000);
  int D = error - _previousError;
  int PIDvalue = (_Kp * _P) + (_Ki * _integral) + (_Kd * D);
  _previousError = error; // Update previous error
  int leftSpeed = _baseSpeed - PIDvalue;
  int rightSpeed = _baseSpeed + PIDvalue;
  motor_control(leftSpeed, rightSpeed);

}


// MOTOR CONTROL CODE
void motor_control(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
      analogWrite(MOTOR_A2, leftSpeed);
      analogWrite(MOTOR_A1, 0);
    } else {
      analogWrite(MOTOR_A2, 0);
      analogWrite(MOTOR_A1, abs(leftSpeed));
    }
    if (rightSpeed >= 0) {
      analogWrite(MOTOR_B1, rightSpeed);
      analogWrite(MOTOR_B2, 0);
    } else {
      analogWrite(MOTOR_B1, 0);
      analogWrite(MOTOR_B2, abs(rightSpeed));
    }

  strip.show();
}

// BASIC MOVES FOR THE OBJECT AVOIDANCE
void moveForward(int delayTime) { 
    analogWrite(MOTOR_A1, 0); 
    analogWrite(MOTOR_A2, 255); 
    analogWrite(MOTOR_B1, 255); 
    analogWrite(MOTOR_B2, 0); 
    delay(delayTime);
}


void turnRight( int delayTime ) { 
    analogWrite(MOTOR_A1, 0); 
    analogWrite(MOTOR_A2, _baseSpeed);  
    analogWrite(MOTOR_B1, 0); 
    analogWrite(MOTOR_B2, _baseSpeed); 
    delay( delayTime );
} 

void turnLeft( int delayTime ) { 
    analogWrite(MOTOR_A1, _baseSpeed); 
    analogWrite(MOTOR_A2, 0); 
    analogWrite(MOTOR_B1, _baseSpeed);  
    analogWrite(MOTOR_B2, 0);
    delay( delayTime ); 
} 

void stopMotors( int delayTime) { 
    analogWrite(MOTOR_A1, 0); 
    analogWrite(MOTOR_A2, 0); 
    analogWrite(MOTOR_B1, 0); 
    analogWrite(MOTOR_B2, 0); 
    delay( delayTime);
}

// NEOPIXEL CODE
void neoPixelColorChange( String color ) {
  if (color == "red") {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 255, 0));  // Red color
    }
  } else if (color == "blue") {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 255));  // Blue color
    }
  } else if (color == "green") {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255, 0, 0));  // Green color
    }
  } else if (color == "orange") {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(80, 255, 0));  // Orange color
    }
  } else if (color == "white") {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(255, 255, 255));  // Orange color
    }
  } else {
    // Default case if color is not matched
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));  // Turn off the pixels
    }
  }
}

// CALIBRATION CODE
int calibrate() {

  int minReading = 1023;  // Start with the highest possible value
  int maxReading = 0;     // Start with the lowest possible value

  unsigned long calibrationTime = millis();

  while(millis() - calibrationTime < 1500) {
    neoPixelColorChange("orange");
    motor_control(_baseSpeed, _baseSpeed);
    // Read sensor values and find min/max
    for (int i = 0; i < 8; i++) {
      _sensorValues[i] = analogRead(sensorPins[i]);

      // Update the min and max readings
      if (_sensorValues[i] < minReading) {
        minReading = _sensorValues[i];
      }
      if (_sensorValues[i] > maxReading) {
        maxReading = _sensorValues[i];
      }
    }
  }

  // Calculate the threshold as the average of min and max
  int threshold = (minReading + maxReading) / 2;

  stopMotors(1000);
  neoPixelColorChange("blue");
  pickUpCone();
  turnLeft(600);
  stopMotors(1000);
  
   return threshold; 
}

void pickUpCone()
{
  moveServo(0);
}

//CODE TO FINISH THE RACE
void exitRace() {
  // Change color to blue
  neoPixelColorChange("green");
  unsigned long stopStartTime = millis();
  while (millis() - stopStartTime < 1000) {
    motor_control(0, 0); // Move backward at -baseSpeed
    // Wait for 1000 milliseconds without blocking the rest of the program
  }
  // Move forward at +baseSpeed for 0.25 seconds
  neoPixelColorChange("orange");
  // Move forward at +baseSpeed
  unsigned long forwardStartTime = millis();
  while (millis() - forwardStartTime < 250) {
    // Wait for 0.25 seconds without blocking the rest of the program
    // This will ensure that motors move forward for the correct time
    motor_control(_baseSpeed, _baseSpeed);
  }

  // After 0.25 seconds, stop the motors and reverse for 1000 milliseconds
  unsigned long stopStartTime2 = millis();
  while (millis() - stopStartTime2 < 1000) {
    motor_control(0, 0); // Move backward at -baseSpeed
    // Wait for 1000 milliseconds without blocking the rest of the program
  }

  unsigned long reverseStartTime = millis();
  while (millis() - reverseStartTime < 1000) {
    motor_control(-_baseSpeed, -_baseSpeed);
    if(millis() - reverseStartTime == 250)
    {
      moveServo(180);
      neoPixelColorChange("blue");
    }      
  }
  
  celebrate();
}


// CODE TO GET THE DISTANCE WITH THE ULTRASONIC SENSORS
int getDistance() {
  long duration;
  int distance;
  // Ensure the trigger pin is LOW before sending a pulse
  digitalWrite(TRIG_PIN, LOW);  // HIGH = OFF (false)
  // Send a 10-microsecond LOW pulse to trigger the sensor
  digitalWrite(TRIG_PIN, HIGH);  // LOW = ON (true) -> Starts the ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);  // HIGH = OFF (false) -> Ends the pulse
  // Measure the time taken for the echo to return
  duration = pulseIn(ECHO_PIN, HIGH);  // Wait for LOW (true) signal to measure time
  // Convert duration to distance (in cm)
  // Speed of sound = 0.034 cm/µs, divide by 2 since sound travels to and from the object
  distance = duration * 0.034 / 2;

  return distance;  // Return the measured distance in cm
}

void obstacleAvoid() {
  for (int i = 0; i < 10; i++) {
    switch(i) {
      case 0:
      turnLeft(actionInterval[i]);  // Pass delay for left turn
      break;
      case 1:
      stopMotors(actionInterval[i]);  // Stop motors
      break;
      case 2:
      moveForward(actionInterval[i]);  // Pass delay for forward movement
      break;
      case 3:
      stopMotors(actionInterval[i]);  // Stop motors
      break;
      case 4:
      turnRight(actionInterval[i]);  // Pass delay for right turn
      break;
      case 5:
      stopMotors(actionInterval[i]);  // Stop motors
      break;
      case 6:
      moveForward(actionInterval[i]);  // Pass delay for forward movement
      break;
      case 7:
      turnRight(actionInterval[i]);
      break;
      case 8:
      stopMotors(actionInterval[i]); 
      break;
      case 9:
      lineSearch();   
    }
    avoidingObstacle = false;     
  }
}

// CODE TO MAKE THE GRIPPER OPEN AND CLOSE
void moveServo(int angle) {
  int pulseWidth = map(angle, 0, 180, 1000, 2000);  // Map angle to servo pulse width
  // Generate a 50Hz signal for ~1 second to move the servo
  for (int i = 0; i < 50; i++) { 
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth); 
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - pulseWidth); 
  }
}

// CELEBRATION CODE FOR WHEN WE FINISH THE RACE
void celebrate() {
  unsigned long celebrateStartTime = millis();
  // Spin for 3 seconds
  while(millis() - celebrateStartTime < 6000) {
    // Spin in place 
    motor_control(_baseSpeed, -_baseSpeed); 
    if(millis() - celebrateStartTime == 500) {
      neoPixelColorChange("orange");
    }
    if(millis() - celebrateStartTime == 1000) {
      neoPixelColorChange("blue");
    }
    if(millis() - celebrateStartTime == 1500) {
      neoPixelColorChange("red");
    }
    if(millis() - celebrateStartTime == 2000) {
          neoPixelColorChange("green");
    }
    if(millis() - celebrateStartTime == 2500) {
      neoPixelColorChange("orange");
    }
    if(millis() - celebrateStartTime ==3000) {
      neoPixelColorChange("blue");    
    }
    if(millis() - celebrateStartTime == 3500) {
      neoPixelColorChange("orange");
    }
    if(millis() - celebrateStartTime == 4000) {
      neoPixelColorChange("blue");
    }
    if(millis() - celebrateStartTime == 4500) {
      neoPixelColorChange("red");
    }
    if(millis() - celebrateStartTime == 5000) {
      neoPixelColorChange("green");
    }
    if(millis() - celebrateStartTime == 5500) {
      neoPixelColorChange("orange");
    }
    if(millis() - celebrateStartTime ==6000)
    {
      neoPixelColorChange("off");
    }   
  }
  // Stop motors after celebration
  motor_control(0, 0);
  isRaceExit = true;  // Ensure race is marked as finished
}

//CODE TO SEARCH THE LINE AFTER THE OBJECT AVOIDANCE
void lineSearch() {
  unsigned long lineSearchStart = millis();
  unsigned long lastSensorCheck = 0;
  while (millis() - lineSearchStart < 2000) { 
    // Check sensors elke 10 ms
    if (millis() - lastSensorCheck >= 5) {
      lastSensorCheck = millis();
      int position = readSensors();
      if (position != 0) {
        break;
      }
      motor_control(_baseSpeed, _baseSpeed);  // Blijf rijden totdat lijn wordt gevonden
    }
  }
}

// STARTUP CODE
void startUp() {
  unsigned long startUpTimer = millis();
  moveServo(0);
  while(millis()-startUpTimer < 1500) {  
    motor_control(0, 0);
    moveServo(170);
    neoPixelColorChange("white");
  }

  startUpCompleted = true;
}