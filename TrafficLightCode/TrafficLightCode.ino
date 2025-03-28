const int ledPin1 = 13; 

const int ledPin2 = 12; 

const int ledPin3 = 11; 

const int buttonPin1 = 2; 

const int buttonPin2 = 3; 

int buttonState1 = 0; 

int buttonState2 = 0; 

unsigned long timerOne = 0; 

bool state; 

bool pressed = false; 

bool pressed1 = false; // Tracks if button is pressed 

  

void setup() { 

  pinMode(ledPin1, OUTPUT); 

  pinMode(ledPin2, OUTPUT); 

  pinMode(ledPin3, OUTPUT); 

  pinMode(buttonPin1, INPUT); 

  pinMode(buttonPin2, INPUT); //Fix: Use internal pull-up 

} 

  

void loop() { 

  blink(1000); 

  buttonState1 = digitalRead(buttonPin1); 

  buttonState2 = digitalRead(buttonPin2); 

  if ( buttonState1 == LOW) { // Fix: Correct syntax & inverted logic (pull-up) 
    
    pressed = true; 

  } 

   

  if(buttonState2 == LOW){ 

    pressed1 = true; 

  } 

  if (pressed) { 

    trafficLight(); 

    pressed = false; 

  } 

  if (pressed1){ 

    blink(200); 
  } 

} 

void trafficLight() { 

  digitalWrite(ledPin1, HIGH); 

  digitalWrite(ledPin2, HIGH); 

  digitalWrite(ledPin3, HIGH); 

  digitalWrite(ledPin1, LOW); // Red light 

  delay(3000); 

  digitalWrite(ledPin1, HIGH); 
  
  digitalWrite(ledPin2, LOW); // Yellow light 

  delay(2000); 

  digitalWrite(ledPin2, HIGH); 

  digitalWrite(ledPin3, LOW); // Green light 

  delay(3000); 

  digitalWrite(ledPin3, HIGH); 
} 

void blink(int INTERVAL){ 

  if(millis() >= timerOne){ 

    timerOne = millis() + INTERVAL; 

    state = !state; 

    digitalWrite(ledPin1, state); 

    digitalWrite(ledPin2, state); 

    digitalWrite(ledPin3, state); 

  } 

} 
