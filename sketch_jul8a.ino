
// Motor 1 pins
const int motor1Pin1 = 7;
const int motor1Pin2 = 5;
const int enable1Pin = 3;

// Motor 2 pins
const int motor2Pin1 = 10;
const int motor2Pin2 = 11;
const int enable2Pin = 9;

// RC receiver channels
const int channel1Pin = 6; // Adjust this pin according to your setup
const int channel2Pin = 2; // Adjust this pin according to your setup

// Desired speeds (0 to 255 for PWM control)
unsigned long channel1Val = 0;
unsigned long channel2Val = 0;

// Function declarations
void moveForward(unsigned long speedValue);
void moveBackward(unsigned long speedValue);
void turnRight(unsigned long speedValue, unsigned long speedValue1);
void turnLeft(unsigned long speedValue, unsigned long speedValue1);
void backright(unsigned long speedValue, unsigned long speedValue1);
void backleft(unsigned long speedValue, unsigned long speedValue1);
void handleCommands(int command, int command1);
void stopMotors();
void setMotorSpeeds(int speed1, int speed2, bool forward);

void setup() {
  Serial.begin(9600);

  // Motor 1 pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  // Motor 2 pins
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // RC receiver channels
  pinMode(channel1Pin, INPUT);
  pinMode(channel2Pin, INPUT);
}

void loop() {
  channel1Val = 0;
  channel2Val = 0;

  // Read values 5 times to get an average
  for (int i = 0; i < 5; i++) {
    channel1Val += pulseIn(channel1Pin, HIGH);
    channel2Val += pulseIn(channel2Pin, HIGH);
    delay(10);  // Small delay to stabilize readings
  }

  // Calculate the average
  channel1Val /= 5;
  channel2Val /= 5;

  // Control motors based on channel values
  if ((channel1Val > 1600) && (channel2Val > 1400 && channel2Val < 1600)) {  // Move forward
    moveForward(channel1Val);
  } else if ((channel1Val < 1400 && channel1Val > 900) && (channel2Val > 1400 && channel2Val < 1600)) {  // Move backward
    moveBackward(channel1Val);
  } else if (channel2Val > 1600 && channel1Val > 1600) {  // Turn right
    turnRight(channel2Val, channel1Val);
  } else if (channel2Val < 1400 && channel1Val > 1600) {  // Turn left
    turnLeft(channel2Val, channel1Val);
  } else if ((channel1Val < 1400 && channel1Val > 900) && (channel2Val > 1600)) {
    backright(channel2Val, channel1Val);
  } else if ((channel1Val < 1400 && channel1Val > 900) && (channel2Val < 1400)) {
    backleft(channel2Val, channel1Val);
  } else if (Serial.available() > 0) {
    int command = Serial.parseInt();
    if (Serial.available() > 0) {
     int command1 = Serial.parseInt();
      handleCommands(command, command1);
    }
  }
  
   else {  // Stop
    stopMotors();
  }
}

void handleCommands(int command, int command1) {
  Serial.println(command);
  Serial.println(command1);
  
  if (command==100 && command1==100) {
    setMotorSpeeds1(100,100 ,false);
   


  }
  else if (command==0 && command1==0){
  stopMotors();
  

  }
  else if (command >0 && command1>0);
  {
    if (command1==50){
    setMotorSpeeds(command,command1 ,true);
    }
    else if (command==50){
      setMotorSpeeds(command,command1 ,true);

    }

  }
  


//   if ((command > 650 && command < 700) || (command == 0)) {
//     stopMotors();
//     Serial.println("s");
//   } else if (command < 650) {
//     if (command1 > 0) {
//       int motorSpeed1 = map(command1, 0, 640, 100, 220);
//       setMotorSpeeds(100, motorSpeed1, true);
//       Serial.println("back LEFT.");
//       Serial.println(motorSpeed1);
//     } else if (command1 < 0) {
//       int motorSpeed2 = map(command1, 0, -640, 100, 220);
//       setMotorSpeeds(motorSpeed2, 100, false);
//       Serial.println("back right.");
//       Serial.println(motorSpeed2);
//     } else {
//       setMotorSpeeds(100, 100, true);
//       Serial.println("f");
//     }
//   } else if (command > 700) {
//     int motorSpeed = map(command, 720, 670, 70, 0);
//     setMotorSpeeds(motorSpeed, motorSpeed, true);
//     Serial.println("Motors moving backward.");
//  }
 }

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, 0);

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, 0);
}

void setMotorSpeeds(int speed1, int speed2, bool forward) {
  if ((forward) && (speed2==50)) {
  //    Serial.println(speed1);
  // Serial.println(speed2);
    digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, speed2);

  // Motor 2 backward
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable2Pin, speed1);

 // Serial.println("backward");
  } 
  else {
      digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enable1Pin, speed2);

  // Motor 2 forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, speed1);

  Serial.println("backright");

  }
}

void setMotorSpeeds1(int speed1, int speed2, bool forward){

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enable1Pin,speed1);

  // Motor 2 backward
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable2Pin, speed2);


}


void moveForward(unsigned long speedValue) {
  int motorSpeed = map(speedValue, 1600, 2000, 0, 255);
  int mb=motorSpeed;
  Serial.println(motorSpeed);
  Serial.println(mb);

  // Motor 1 forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, motorSpeed);

  // Motor 2 forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, motorSpeed);

  Serial.println("forward");
}

void moveBackward(unsigned long speedValue) {
  int motorSpeed = map(speedValue, 1400, 1000, 0, 255);
  int mb=motorSpeed;
  Serial.println(motorSpeed);
  Serial.println(mb);

  // Motor 1 backward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enable1Pin, motorSpeed);

  // Motor 2 backward
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable2Pin, motorSpeed);

  Serial.println("backward");
}

void turnRight(unsigned long speedValue, unsigned long speedValue1) {
  int motorSpeed = map(speedValue1, 1600, 2000, 0, 255);
  int motorSpeed2 = map(speedValue, 1600, 2000, motorSpeed, 0);
  Serial.println(motorSpeed);
  Serial.println(motorSpeed2);

  // Motor 1 forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, motorSpeed2);

  // Motor 2 forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, motorSpeed);

  Serial.println("Turningright");
}

void turnLeft(unsigned long speedValue, unsigned long speedValue1) {
  int motorSpeed2 = map(speedValue1, 1600, 2000, 0, 255);
  int motorSpeed = map(speedValue, 1400, 1000, motorSpeed2, 0);

  Serial.println(motorSpeed2);
  Serial.println(motorSpeed);

  // Motor 1 forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, motorSpeed2);

  // Motor 2 forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, motorSpeed);

  Serial.println("Turningleft");
}

void backright(unsigned long speedValue, unsigned long speedValue1) {
  int motorSpeed2 = map(speedValue1, 1400, 1000, 0, 255);
  int motorSpeed = map(speedValue, 1600, 2000, 0, 150);

  Serial.println(motorSpeed2);
  Serial.println(motorSpeed);

  // Motor 1 backward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enable1Pin, motorSpeed2);

  // Motor 2 forward
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, motorSpeed);

  Serial.println("backright");
}

void backleft(unsigned long speedValue, unsigned long speedValue1) {
  int motorSpeed2 = map(speedValue1, 1400, 1000, 0, 255);
  int motorSpeed = map(speedValue, 1400, 1000, 0, 150);

  Serial.println(motorSpeed2);
  Serial.println(motorSpeed);

  // Motor 1 forward
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, motorSpeed);

  // Motor 2 backward
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable2Pin, motorSpeed2);

  Serial.println("backleft");
}