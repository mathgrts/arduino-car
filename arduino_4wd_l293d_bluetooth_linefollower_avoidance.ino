/*
   created by: Muhammad Ilmi Maulana June 2022
   ini merupakan code untuk arduino 4-2wd car
   mengunakan motor shield l293d
   terdapat 4 mode: manual override using BT, line follower, obstacle avoidance, table edge avoidance

*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// lcd ukuran 4x20 address i2c nya = 0x27 juga
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//===================================================================================
////Arduino PWM Speed Control
////this part is for adafruit motorshield v1.2 only
//int E1 = 5;
//int M1 = 4;
//int E2 = 6;
//int M2 = 7;
//===================================================================================
//this part is for l293d motorshield, choose wisely..
#include <MotorDriver.h>
MotorDriver m;
#define motor1 1 //port motor # pada motor shield
#define motor2 4
#define freePin1 6 //pin 6 tidak digunakan jika tidak menggunakan port motor #3 pada shield
#define freePin2 3 //pin 3 tidak digunakan jika tidak menggunakan port motor #2 pada shield
//===================================================================================

int speedCar = 100;     // 50 - 255.
int speed_Coeff = 50; //divider speed untuk belok sambil maju/mundur. bigger value=sharpTurn

const int pinButton = 2;
int selector = 1; //initialize which mode start after boot 1=BT 2=Line 3=Obstacle 4=Table
boolean isPressed = false;
long prevMillis;
#define delayLoop 100 //milisecond

//===================================================================================

int command;             //Int to store app command state.

boolean lightFront = false;
boolean lightBack = false;
boolean horn = false;
boolean genit = false;

#define pinFrontLight 13
#define pinBuzzer 9 //hati-hati pin 9&10 digunakan utk servo
#define buzzOn HIGH
#define buzzOff LOW

//===================================================================================

#define pinLeftSensor 15
#define pinRightSensor 14
int speedLine = 100;
int speedLineBelok = 150;

//===================================================================================
#include <Servo.h>        // Include Servo Library
#include <NewPing.h>      // Include Newping Library

#define pinServo 10
#define pinTrigger  16  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define pinEcho     17  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define maxDistance 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 250cm.

Servo servo_motor;  // Servo's name
NewPing sonar(pinTrigger, pinEcho, maxDistance); // NewPing setup of pins and maximum distance.

boolean goesForward = false;
int distance = 100;

int speedObstacle = 100;
int speedObstacleBelok = 100;
//===================================================================================

//#include <IRremote.h>
//IRrecv IR(A2);
//decode_results result;
//
//#define up 1033561079
//#define down 465573243
//#define left 2351064443
//#define right 71952287
//#define Stop 1217346747

//===================================================================================

void setup() {

  //this part is for adafruit motorshield v1.2 only
//  pinMode(M1, OUTPUT);
//  pinMode(M2, OUTPUT);

  lcd.init();                      // initialize the lcd
  //  lcd.init();
  lcd.noBacklight(); //save battery power by turning off the backlight
  printLCD(); //print for initialization
  
  Serial.begin(9600);

  pinMode(pinButton, INPUT_PULLUP);
  pinMode(pinLeftSensor, INPUT);
  pinMode(pinRightSensor, INPUT);

  pinMode(pinFrontLight, OUTPUT);
  //pinMode(pinBackLight, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, buzzOff);

  //  IR.enableIRIn();

  servo_motor.attach(pinServo);   // Attachs the servo on pin 9 to servo object.
  servo_motor.write(115);   // Set at 115 degrees.
  delay(2000);              // Wait for 2s.
  distance = readPing();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);


  prevMillis = millis();
}

void loop() {
  //  selector = 1;
  //  /*
  if (millis() - prevMillis > 100) {
    if (digitalRead(pinButton) == LOW && isPressed == false ) //button is pressed AND this is the first digitalRead() that the button is pressed
    {
      isPressed = true;  //set to true, so this code will not run again until button released
      //Serial.println(selector);

      servo_motor.write(115);
      digitalWrite(pinBuzzer, buzzOff);
      stopCar();

      selector++; // this is done after the doSwitchStatement(), so case 0 will be executed on the first button press
      if (selector >= 5) {
        selector = 1;
      }
      // selector = (selector+1) % 4;  // does the same, without if-statement

      printLCD();

    } else if (digitalRead(pinButton) == HIGH)
    {
      isPressed = false; //button is released, variable reset
    }
    prevMillis = millis();
  }
  //*/
  doSwitchStatement(); // a call to a separate function that performs the switch statement and subsequent evoked code

  //delay(delayLoop);

}

void doSwitchStatement() {
  switch (selector) {
    case 1:
      //Serial.println("MODE_BT_CONTROL");
      modeBTControl();
      break;
    case 2:
      //Serial.println("MODE_LINE_FOLLOWER");
      modeLineFollower();
      break;
    case 3:
      //Serial.println("MODE_AVOIDANCE");
      modeObstacleAvoidance();
      break;
    case 4:
      //Serial.println("MODE_TABLE");
      modeTableEdgeAvoidance();
      break;
  }
}

void printLCD() {

  switch (selector) {
    case 1:
      //Serial.println("MODE_BT_CONTROL");
      lcd.setCursor(0, 0); //x,y
      lcd.print("   BLUETOOTH    ");
      lcd.setCursor(0, 1); //x,y
      lcd.print("    CONTROL     ");
      break;
    case 2:
      //Serial.println("MODE_LINE_FOLLOWER");
      lcd.setCursor(0, 0); //x,y
      lcd.print("      LINE      ");
      lcd.setCursor(0, 1); //x,y
      lcd.print("    FOLLOWER    ");
      break;
    case 3:
      //Serial.println("MODE_AVOIDANCE");
      lcd.setCursor(0, 0); //x,y
      lcd.print("   OBSTACLE     ");
      lcd.setCursor(0, 1); //x,y
      lcd.print("   AVOIDANCE    ");
      break;
    case 4:
      //Serial.println("MODE_TABLE");
      lcd.setCursor(0, 0); //x,y
      lcd.print("   TABLE EDGE   ");
      lcd.setCursor(0, 1); //x,y
      lcd.print("   AVOIDANCE    ");
      break;
  }
}

void modeTableEdgeAvoidance() {
  int LEFT_SENSOR = digitalRead(pinLeftSensor);
  int RIGHT_SENSOR = digitalRead(pinRightSensor);

  if (RIGHT_SENSOR == LOW && LEFT_SENSOR == LOW ) { //FORWARD
    speedCar = 100;
    goAhead();
  }
  else if (RIGHT_SENSOR == LOW && LEFT_SENSOR == HIGH) { //LEFT
    speedCar = 100;
    goBack();
    delay(700);
    speedCar = 150;
    goLeft();
    delay(800);
  } else if (RIGHT_SENSOR == HIGH && LEFT_SENSOR == false) { //RIGHT
    speedCar = 100;
    goBack();
    delay(700);
    speedCar = 150;
    goRight();
    delay(800);
  }
  else { //STOP
    brakeCar(); //brake only work with l293d motorshield
    delay(100);
    stopCar();
  }
}

void modeLineFollower() {
  int LEFT_SENSOR = digitalRead(pinLeftSensor);
  int RIGHT_SENSOR = digitalRead(pinRightSensor);

  //  if (distance <= 10)  {
  //    stopCar();
  //  } else {

  if (RIGHT_SENSOR == LOW && LEFT_SENSOR == LOW) { //FORWARD
    speedCar = 70;
    goAhead();
  } else if (RIGHT_SENSOR == HIGH && LEFT_SENSOR == LOW) { //LEFT
    speedCar = 130;
    goLeft();
    delay(400);
  } else if (RIGHT_SENSOR == LOW && LEFT_SENSOR == HIGH) { //RIGHT
    speedCar = 130;
    goRight();
    delay(400);
  } else { //STOP
    stopCar();
  }

  //  }
  //  distance = readPing();

}

void modeObstacleAvoidance() {

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 30)
  {
    digitalWrite(pinBuzzer, buzzOn);
    speedCar = 100;
    stopCar();
    delay(300);
    goBack();
    delay(700);
    stopCar();
    delay(300);
    digitalWrite(pinBuzzer, buzzOff);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distanceRight >= distanceLeft)
    {
      speedCar = 150;
      goRight();
      delay(800);
      stopCar();
    }
    else
    {
      speedCar = 150;
      goLeft();
      delay(800);
      stopCar();
    }

  }
  else
  {
    speedCar = 100;
    goAhead();
  }

  distance = readPing();
}

void modeBTControl() {
  if (Serial.available() > 0) {
    command = Serial.read();
    stopCar();       //Initialize with motors stopped.

    if (lightFront) {
      digitalWrite(pinFrontLight, HIGH);
    }
    if (!lightFront) {
      digitalWrite(pinFrontLight, LOW);
    }
    //    if (lightBack) {
    //      digitalWrite(pinBackLight, HIGH);
    //    }
    //    if (!lightBack) {
    //      digitalWrite(pinBackLight, LOW);
    //    }
    if (horn) {
      digitalWrite(pinBuzzer, buzzOn);
    }
    if (!horn) {
      digitalWrite(pinBuzzer, buzzOff);
    }
    if (genit) {
      servo_motor.write(speedCar / 1.5);
    }
    if (!genit) {
      servo_motor.write(115);
    }

    switch (command) {
      case 'F': goAhead(); break;
      case 'B': goBack(); break;
      case 'L': goLeft(); break;
      case 'R': goRight(); break;
      case 'I': goAheadRight(); break;
      case 'G': goAheadLeft(); break;
      case 'J': goBackRight(); break;
      case 'H': goBackLeft(); break;
      case '0': speedCar = 100; break;
      case '1': speedCar = 115; break;
      case '2': speedCar = 130; break;
      case '3': speedCar = 145; break;
      case '4': speedCar = 160; break;
      case '5': speedCar = 175; break;
      case '6': speedCar = 190; break;
      case '7': speedCar = 205; break;
      case '8': speedCar = 220; break;
      case '9': speedCar = 235; break;
      case 'q': speedCar = 255; break;
      case 'W': lightFront = true; break;
      case 'w': lightFront = false; break;
      //case 'U': lightBack = true; break;
      //case 'u': lightBack = false; break;
      case 'X': genit = true; break;
      case 'x': genit = false; break;
      case 'V': horn = true; break;
      case 'v': horn = false; break;

    }
  }
}

int lookRight()     // Look Right Function for Servo Motor
{
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft()      // Look Left Function for Servo Motor
{
  servo_motor.write(180);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int readPing()      // Read Ping Function for Ultrasonic Sensor.
{
  delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}

///*
  void goAhead() {
  m.motor(motor1, FORWARD, speedCar);
  m.motor(motor2, FORWARD, speedCar);
  }
  void goBack() {
  m.motor(motor1, BACKWARD, speedCar);
  m.motor(motor2, BACKWARD, speedCar);
  }
  void goRight() {
  m.motor(motor1, FORWARD, speedCar);
  m.motor(motor2, BACKWARD, speedCar);
  }
  void goLeft() {
  m.motor(motor1, BACKWARD, speedCar);
  m.motor(motor2, FORWARD, speedCar);
  }
  void goAheadRight() {
  m.motor(motor1, FORWARD, speedCar);
  m.motor(motor2, FORWARD, speedCar / speed_Coeff);
  }
  void goAheadLeft() {
  m.motor(motor1, FORWARD, speedCar / speed_Coeff);
  m.motor(motor2, FORWARD, speedCar);
  }
  void goBackRight() {
  m.motor(motor1, BACKWARD, speedCar);
  m.motor(motor2, BACKWARD, speedCar / speed_Coeff);
  }
  void goBackLeft() {
  m.motor(motor1, BACKWARD, speedCar / speed_Coeff);
  m.motor(motor2, BACKWARD, speedCar);
  }
  void stopCar() {
  m.motor(motor1, RELEASE, 0);
  m.motor(motor2, RELEASE, 0);
  }
  void brakeCar() {
  m.motor(motor1, BRAKE, 150);
  m.motor(motor2, BRAKE, 150);
  }
//*/

/*
void goAhead() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar);
}
void goBack() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar);
}
void goRight() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar);
}
void goLeft() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar);
}
void goAheadRight() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar / speed_Coeff);
}
void goAheadLeft() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, speedCar / speed_Coeff);
  analogWrite(E2, speedCar);
}
void goBackRight() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, speedCar);
  analogWrite(E2, speedCar / speed_Coeff);
}
void goBackLeft() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, speedCar / speed_Coeff);
  analogWrite(E2, speedCar);
}
void stopCar() {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}
void brakeCar() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, 2);
  analogWrite(E2, 2);
}
*/

/*
  void modeIRRemote() {
      if (IR.decode(&result)) {
        Serial.println(result.value);
        IR.resume();
      }
      delay(100);
      if (result.value == up ) {
        maju();
      } else if (result.value == down ) {
        mundur();
      } else if (result.value == left) {
        belokKiri();
      } else if (result.value == right) {
        belokKanan();
      } else if (result.value == Stop) {
        berhenti();
      }
  }

*/
