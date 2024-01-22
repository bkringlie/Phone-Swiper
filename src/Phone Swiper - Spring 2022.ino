// Author Names: Theodore Wilson Mason Greseth Brandt Kringlie
// Date: April 13, 2021
// Short Description: The set up of our code takes data from two lidar sensors and has the data interact with a double jointed servo setup (two servos: one glue onto the side of the other)...
//                    The goal of the code is to wait til one of the lidar sensors registers a distance that is less than 100mm, and if so the servos execute a swipe in the right or left direction...
//                    depending on which lidar is triggered. After a swipe is executed the servos go back to resting position waiting to be triggered again.
// References: 
  // * Pololu Library Reference (to find get and setAddress commands)
  //   --> https://github.com/pololu/vl53l1x-arduino
  // * David Orser Reference Code (Used as a template for our final code)
  //   --> Continuous_TwoLIDAR_v002.ino

#include <Wire.h>
#include <VL53L1X.h>  // Library by Pololu - Version 1.3.1
#include <Servo.h>    // Library built into Arduino - Version 1.1.8

VL53L1X gVL531X;

VL53L1X sensor;                     // Right Lidar sensor
VL53L1X sensor2;                    // Left Lidar sensor

Servo servo0ne;                     //Stationary Servo Motor
Servo servoTwo;                     //Servo Motor that is attached to servo0ne and moves up and down

int servoPin = 9;                   // Pin that the servo1 is connected to
int servoPos = 135;                 // initial servo1 position
int servo2Pin = 8;                  // Pin that the servo2 is connected to
int servo2Pos = 90;                 // initial servo2 position

void setup()
{
  servo0ne.attach(servoPin);
  servoTwo.attach(servo2Pin);
  servo0ne.write(servoPos);
  servoTwo.write(servo2Pos);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  delay(500);
  
  Wire.begin();
  Serial.begin (9600);

  pinMode(2, INPUT);
  delay(150);
  sensor.init(true);
  delay(100);
  
  sensor.setAddress((uint8_t)22);   // Line 1278 in VL53L1X.h Library by Polulu
  sensor.setTimeout(500);           // Line 1314 in VL53L1X.h Library by Polulu

  pinMode(3, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  
  sensor2.setAddress((uint8_t)25);  // Line 1278 in VL53L1X.h Library by Polulu
  sensor2.setTimeout(500);          // Line 1314 in VL53L1X.h Library by Polulu
  
  Serial.println("addresses set");
  sensor.startContinuous(30);       // Line 1301 in VL53L1X.h Library by Polulu
  sensor2.startContinuous(30);      // Line 1301 in VL53L1X.h Library by Polulu
  
  Serial.println("start read range");
  gVL531X.setTimeout(1000);         // Line 1314 in VL53L1X.h Library by Polulu
  
  servo0ne.write(servoPos);         // Resting Position of Servo One
  servoTwo.write(servo2Pos);        // Resting Position of Servo Two
}

void loop()
{
  int dist1;
  int dist2;
  
  servo0ne.write(servoPos);         // sets position back to original position
  servoTwo.write(servo2Pos);        // sets position back to original position

  // records distance of each lidar in mm every 30 ms
  dist1 = sensor.readRangeContinuousMillimeters();  // Line 1304 in VL53L1X.h Library by Polulu
  dist2 = sensor2.readRangeContinuousMillimeters(); // Line 1304 in VL53L1X.h Library by Polulu

  // Prints data onto Serial monitor for visualization
  Serial.print("LEFT SENSOR: ");
  Serial.print(dist1);
  Serial.print(" [mm]   ");
  Serial.print("RIGHT SENSOR: ");
  Serial.print(dist2);
  Serial.println(" [mm]   ");
  delay(30);

  // if hand is detected within 100mm of right Lidar Sensor 
  // lidar, left swipe is executed
  if (dist1 < 100)  {
      Serial.print("LEFT");
      servoTwo.write(135);          // Intialize movement where stylus goes from 90 degrees to 135 degrees
      delay(500);
      servo0ne.write(90);           // The stationary servo brings servoTwo perpendicular with phone screen
      delay(500);
      // Controls speed of servo when executing the swipe
      // ServoTwo performs swipe from 135 degrees to 45 degrees in the LEFT direction.
      for (int pos = 135; pos >= 45; pos -= 2) {
        servoTwo.write(pos);
        delay(10);
      }
      delay(100);
      servo0ne.write(servoPos);     // Stationary servo brings servoTwo to resting position allowing it to reset
      delay(500);
      servoTwo.write(servo2Pos);    // ServoTwo goes back to resting position which is at 90 degrees
    }

  // if hand is detected within 50mm of left Lidar Sensor 
  // lidar, right swipe is executed
  if (dist2 < 100)  {
      Serial.print("RIGHT");
      servoTwo.write(45);           // Intialize movement where stylus goes from 90 degrees to 45 degrees
      delay(500);
      servo0ne.write(90);           // The stationary servo brings servoTwo perpendicular with phone screen
      delay(500);
        // Controls speed of servo when executing the swipe
        // ServoTwo performs swipe from 45 degrees to 135 degrees in the RIGHT direction.
      for (int pos = 45; pos <= 135; pos += 2) {
        servoTwo.write(pos);
        delay(10);
      }
      delay(100);
      servo0ne.write(servoPos);     // Stationary servo brings servoTwo to resting position allowing it to reset
      delay(500);
      servoTwo.write(servo2Pos);    // ServoTwo goes back to resting position which is at 90 degrees
    }

    // Line 1316 in VL53L1X.h Library by Polulu
  if (sensor.timeoutOccurred()){
    Serial.print("sensor1timeout\n");
  }
    // Line 1316 in VL53L1X.h Library by Polulu
  if (sensor2.timeoutOccurred()){
    Serial.print("sensor2timeout\n");
  }
}
