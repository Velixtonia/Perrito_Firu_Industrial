#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//This code helps you determine the usable PCA9685 value range for each servo
//so you can calibrate them correctly.

//I recommend starting with a single leg (Right Front, servos 3, 4, and 5).
//Then use the 'IK3-TEST-OneLeg-RF' sketch to verify that the leg moves correctly.
//Once it works, return to this sketch and calibrate the remaining servos.


//WARNING:
//Remove the linkage or lift the robot off the ground during calibration.
//Moving a servo beyond the mechanical limits may damage the servo or the leg.


//How to use this code?
//take the first servo (you can start with FR tibia - servo5) and find the minimum and maximum PCA9685 values the servo responds to.
//e.g. my RF tibia servo moves up to 510 but does not respond to 520 or higher.
//It also moves down to 120 but not below that.

//Set the servo close to one end of its range and mount the horn so that
//the joint is at its corresponding mechanical limit.
//(e. g. I set the RF tibia servo to 200 and mount it so that it aligns with joint's max angle, in that case 180, meaning femur/tibia completely straight)

//Next, determine which PCA9685 value rotates the joint by exactly 90°
//(or -90°, depending on the joint)
//(e. g. I set the RF tibia servo to 410 and it moves by 90 degrees)

//Once You have these two values, note them down using  #define function down there:
//  #define SERVO_90_5 410 
//  #define SERVO_180_5 200 

//congratulations you set your first servo :) , only 11 left...
//You don't have to be super precise with measuring these values, it's just a DIY project not a military-grade product...

//once You set all of the servos, and note down the values, copy them to the other arduino files



//-------------------------------------------------------PCA9685 SERVO VALUES----------------------------------------------------------

  #define SERVO_0_0 305   // LEFT FRONT hip servo 0       CHANGE IT
  #define SERVO_M90_0 105 //LEFT FRONT hip servo -90      CHANGE IT

  #define SERVO_M90_1 320   // LEFT FRONT femur servo -90 CHANGE IT
  #define SERVO_M180_1 555  // LEFT FRONT femur servo  -180 CHANGE IT

  #define SERVO_90_2 215   // LEFT FRONT tibia servo  90   CHANGE IT
  #define SERVO_180_2 420   // LEFT FRONT tibia servo  180  CHANGE IT


  #define SERVO_0_3 320   // RIGHT FRONT hip servo 0       CHANGE IT
  #define SERVO_M90_3 510   // RIGHT FRONT hip servo -90     CHANGE IT

  #define SERVO_M90_4 360 // RIGHT FRONT femur servo -90     CHANGE IT
  #define SERVO_M180_4 145  // RIGHT FRONT femur servo   -180     CHANGE IT

  #define SERVO_90_5 410   // RIGHT FRONT tibia servo  90    CHANGE IT
  #define SERVO_180_5 200   // RIGHT FRONT tibia servo  180     CHANGE IT


  #define SERVO_0_6 305 //LEFT BACK hip servo 0               CHANGE IT
  #define SERVO_M90_6 500   // LEFT BACK hip servo -90       CHANGE IT

  #define SERVO_M90_7 320   // LEFT BACK femur servo -90 CHANGE IT
  #define SERVO_M180_7 525  // LEFT BACK femur servo  -180 CHANGE IT

  #define SERVO_90_8 250   // LEFT BACK tibia servo  90   CHANGE IT
  #define SERVO_180_8 460   // LEFT BACK tibia servo  180  CHANGE IT


  #define SERVO_0_9 335   // RIGHT BACK hip servo 0       CHANGE IT
  #define SERVO_M90_9 125   // RIGHT BACK hip servo -90     CHANGE IT

  #define SERVO_M90_10 325 // RIGHT BACK femur servo -90     CHANGE IT
  #define SERVO_M180_10 125  // RIGHT BACK femur servo   -180     CHANGE IT

  #define SERVO_90_11 390   // RIGHT BACK tibia servo  90    CHANGE IT
  #define SERVO_180_11 180   // RIGHT BACK tibia servo  180     CHANGE IT


//-------------------------------------------------------PCA9685 SERVO VALUES----------------------------------------------------------


void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(50);     
    Serial.println("Enter servo number (0-11) and Servo value , e.g., 3 320");
}

void loop() {
    if (Serial.available()) {
        int servoNum, servoVal;
        
        // Read the servo number and angle from the terminal
        servoNum = Serial.parseInt();
        servoVal = Serial.parseInt();

            pwm.setPWM(servoNum, 0, servoVal);
            
            Serial.print("Servo ");
            Serial.print(servoNum);
            Serial.print(" set to ");
            Serial.println(servoVal);

        // Clear the serial buffer
        while (Serial.available()) {
            Serial.read();
        }
    }
}

