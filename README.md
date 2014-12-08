#include <Servo.h>

int rPin = 4;
int lPin = 2;

class Robot {
public:
  Robot() {
  }  
  void attachServos(int leftServoPin, int rightServoPin) {
    left.attach(leftServoPin);
    right.attach(rightServoPin);
  }
  void moveForward() {
    left.write(180);
    right.write(180); 
    delay(15);
  }

  void turnLeft() {
    left.write(0);
    right.write(180); 
    delay(15);
  }
  void turnRight() {
    left.write(180);
    right.write(0); 
    delay(15);
  }

  int ping(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin,LOW);
    pinMode(pin,INPUT);

    return pulseIn(pin,HIGH);
  }
private:
  Servo left, right;
};

//class Wanderer : 
//public Robot {
//public:
//  void test() {
//    int a;
//  } 
//};

Robot robot;
//Wanderer robot;

void setup() {
  Serial.begin(9600);
  robot.attachServos(lPin, rPin); 
}

void loop() {
  int threshold = 1000;
  
  int pingTime = robot.ping(6);
//  int oldPingTime = pingTime;

  Serial.println(pingTime);

//  if (pingTime > threshold) {
//    robot.moveForward();  
//  } 
//  else {
//    int startTime = millis();
//    int newTime = millis();
//    while (newTime - startTime < 1000) {
//      robot.turnLeft();
//      newTime = millis();
//    }
//  }
}














