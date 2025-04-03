
const int MOTOR_A2 = 10; //left forward
const int MOTOR_A1 = 11; //left backward

const int MOTOR_B2 = 8; //right BACKWARD
const int MOTOR_B1 = 9; //right FORWARD

const int BUTTON_PIN = 7;
int _buttonState;
bool _pressed = false;

const int ECHO_PIN = 2; 
const int TRIG_PIN = 3;
int _delayTime;


void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT);
  

}
void loop() {

  int distance = getDistance();
 
straight(3000);
 stopMotors(500);
 turnRight(1000);
 stopMotors(500);
 straight(3000);
 stopMotors(500);
 turnLeft(1000);
 stopMotors(500);
 spin(1000); 
 stopMotors(500);
 reverse(3000);
 stopMotors(3000);
 
  
 

 





 
}


void turnLeft(int _delayTime){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2,0 );
  
  analogWrite(MOTOR_B1, 255);
  analogWrite(MOTOR_B2, 0); 
  delay(_delayTime);
}
void  turnRight(int _delayTime){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 255);
  
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0); 
  delay(_delayTime);
}
void reverse( int _delayTime){
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_A2, 0);
  
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 255); 
  delay(_delayTime);
}
void straight( int _delayTime ){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 255);
  
  analogWrite(MOTOR_B1,255);
  analogWrite(MOTOR_B2, 0); 
  delay(_delayTime);
}
void stopMotors( int _delayTime){
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  
  analogWrite(MOTOR_B1,0);
  analogWrite(MOTOR_B2, 0); 
  delay(_delayTime);
}

void spin(int _delayTime)
{
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_A2, 0);
  
  analogWrite(MOTOR_B1, 255);
  analogWrite(MOTOR_B2, 0); 
  delay(_delayTime);
}

int getDistance(){
  int distance;
  long duration;

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, HIGH);
  digitalWrite(TRIG_PIN, LOW );

  duration = pulseIn(ECHO_PIN, HIGH);

  distance = duration * 0.034 / 2;
  return distance;
}
 

