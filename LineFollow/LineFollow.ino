// Motor Pins
const int motor_A2 = 10; // Left forward 
const int motor_A1 = 11; // Left backward 
const int motor_B2 = 8;  // Right backward 
const int motor_B1 = 9;  // Right forward 
const int bluetooth_pin = 1;

// Line sensor array
const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int _sensorValues[8];
int _sensorThreshold = 600;
int _sensorWeights[] =  { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

// Variables for Hysteresis
bool leftOnLine = false;
bool rightOnLine = false;

float _Kp = 0.08;  // Proportional gain (controls how much the robot adjusts based on the error)
float _Ki = 0.001;  // Integral gain (helps correct small, long-term errors)
float _Kd = 0.02;   // Derivative gain (helps dampen sudden changes in error)

int _previousError = 0;
int _P = 0;  // Store the previous error value for calculating the derivative term
int _integral = 0;  // Integral of the error, to help fix long-term errors
int _baseSpeed = 175;

void setup() {
    Serial.begin(9600);

    pinMode(motor_A1, OUTPUT); 
    pinMode(motor_A2, OUTPUT); 
    pinMode(motor_B1, OUTPUT); 
    pinMode(motor_B2, OUTPUT);
    pinMode(bluetooth_pin, OUTPUT);

    for (int i = 0; i < 8; i++) {  
        pinMode(sensorPins[i], INPUT);
    }
}   

void loop() {
    int position = readSensors();
    int error = position;

    PIDcontrol(error);
}

int readSensors() {
    int weightedSum = 0; // Sum of all sensor weights
    int sum = 0; // Number of active sensors

    for (int i = 0; i < 8; i++) {
        _sensorValues[i] = analogRead(sensorPins[i]);

        if (_sensorValues[i] > _sensorThreshold) {
            weightedSum += _sensorWeights[i];
            sum++;
        }
    }

    if (sum == 0) {
        return (_previousError > 0) ? 1000 : -1000;
    }

    return weightedSum / sum;
}

void PIDcontrol(int error) {
    _P = error;
    _integral += error;

    // Prevent integral windup
    _integral = constrain(_integral, -1000, 1000);

    int D = error - _previousError;

    int PIDvalue = (_Kp * _P) + (_Ki * _integral) + (_Kd * D);

    _previousError = error;

    int leftSpeed = _baseSpeed - PIDvalue;
    int rightSpeed = _baseSpeed + PIDvalue;

    motor_control(leftSpeed, rightSpeed);
}

void motor_control(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0) {
        analogWrite(motor_A2, leftSpeed);
        analogWrite(motor_A1, 0);
    } else {
        analogWrite(motor_A2, 0);
        analogWrite(motor_A1, abs(leftSpeed));
    }

    if (rightSpeed >= 0) {
        analogWrite(motor_B1, rightSpeed);
        analogWrite(motor_B2, 0);
    } else {
        analogWrite(motor_B1, 0);
        analogWrite(motor_B2, abs(rightSpeed));
    }
    
}

void moveForward() { 
    analogWrite(motor_A1, 0); 
    analogWrite(motor_A2, 170); 
    analogWrite(motor_B1, 170); 
    analogWrite(motor_B2, 0); 
} 

void turnRight() { 
    analogWrite(motor_A1, 0); 
    analogWrite(motor_A2, 170);  
    analogWrite(motor_B1, 0); 
    analogWrite(motor_B2, 170); 
} 

void turnLeft() { 
    analogWrite(motor_A1, 170); 
    analogWrite(motor_A2, 0); 
    analogWrite(motor_B1, 170);  
    analogWrite(motor_B2, 0); 
} 

void stopMotors() { 
    analogWrite(motor_A1, 0); 
    analogWrite(motor_A2, 0); 
    analogWrite(motor_B1, 0); 
    analogWrite(motor_B2, 0); 
}

