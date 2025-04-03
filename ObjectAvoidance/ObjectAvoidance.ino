// Define motor control pins 
const int motor_A2 = 10; // Left forward 
const int motor_A1 = 11; // Left backward 
const int motor_B2 = 8;  // Right backward 
const int motor_B1 = 9;  // Right forward 

  
// Define ultrasonic sensor pins 
#define ECHO_PIN 2  // Echo pin (input) 
#define TRIG_PIN 3  // Trigger pin (output) 

  
void setup() { 

  // Set motor pins as outputs 
  pinMode(motor_A1, OUTPUT); 
  pinMode(motor_A2, OUTPUT); 
  pinMode(motor_B1, OUTPUT); 
  pinMode(motor_B2, OUTPUT); 

  // Set ultrasonic sensor pins 
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT); 

  // Start serial monitor for debugging 
  Serial.begin(9600); 

} 

void loop() { 

  int distance = getDistance();  // Get the distance from the ultrasonic sensor 

  if(distance > 25)
  { 
    moveForward(); 
  } 
  else if (distance < 25) 
  { 
    stopMotors();
    turnRight();
    delay(350);
    moveForward();
    delay(650);
    turnLeft();
    delay(350);
    moveForward();
    delay(650);
    turnLeft();
    delay(350);
    moveForward();
    delay(650);
    turnRight();
    delay(300); 
  } 
} 

int getDistance() 
{ 
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

// Function to move forward 
void moveForward(){ 
  analogWrite(motor_A1, 0); 
  analogWrite(motor_A2, 255); 
  analogWrite(motor_B1, 255); 
  analogWrite(motor_B2, 0); 
} 

// Function to turn left 
void turnLeft(){ 
  analogWrite(motor_A1, 255); 
  analogWrite(motor_A2, 0); 
  analogWrite(motor_B1, 255); 
  analogWrite(motor_B2, 0); 
} 

// Function to turn right 
void turnRight() { 
  analogWrite(motor_A1, 0); 
  analogWrite(motor_A2, 255); 
  analogWrite(motor_B1, 0); 
  analogWrite(motor_B2, 255); 
} 

// Function to move in reverse 
void reverse() { 
  analogWrite(motor_A1, 255); 
  analogWrite(motor_A2, 0); 
  analogWrite(motor_B1, 0); 
  analogWrite(motor_B2, 255); 
} 

// Function to stop motors 
void stopMotors() { 
  analogWrite(motor_A1, 0); 
  analogWrite(motor_A2, 0); 
  analogWrite(motor_B1, 0); 
  analogWrite(motor_B2, 0); 
} 
