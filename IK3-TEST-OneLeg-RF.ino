#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//How the code works:
//Enter x, y, z coordinates -> compute inverse kinematics -> move the servos
//Made for the RIGHT FRONT leg.


//Before You can use it, You need to know the PCA9685 servo values:
//RF hip 0 -90
//RF femur -90 -180
//RF tibia 90 180
//If you don't know how to do it, open arduino file called 'SetServosPCA9685'


//-------------------------------------------------------PCA9685 SERVO VALUES----------------------------------------------------------

  #define SERVO_0_3 320   // RIGHT FRONT hip servo 0       CHANGE IT
  #define SERVO_M90_3 510   // RIGHT FRONT hip servo -90     CHANGE IT

  #define SERVO_M90_4 360 // RIGHT FRONT femur servo -90     CHANGE IT
  #define SERVO_M180_4 145  // RIGHT FRONT femur servo   -180     CHANGE IT

  #define SERVO_90_5 410   // RIGHT FRONT tibia servo  90    CHANGE IT
  #define SERVO_180_5 200   // RIGHT FRONT tibia servo  180     CHANGE IT

//-------------------------------------------------------PCA9685 SERVO VALUES----------------------------------------------------------

void setup() {
Serial.begin(115200);
pwm.begin();
pwm.setPWMFreq(50);
Serial.println("Enter target foot position (x y z) in cm:");
}

void loop() {
  static String input = "";

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        float footX, footY, footZ;

        // Input format: x y z
        // e.g. 5 -10 -2
        int space1 = input.indexOf(' ');
        int space2 = input.indexOf(' ', space1 + 1);
        if (space1 > 0 && space2 > space1) {
          footX = input.substring(0, space1).toFloat();
          footY = input.substring(space1 + 1, space2).toFloat();
          footZ = input.substring(space2 + 1).toFloat();

          computeRFLegIK(footX, footY, footZ);
        } else {
          Serial.println("Invalid input. Use format: x y z");
        }

        Serial.println("\nEnter target foot position (x y z) in cm:");
        input = "";
      }
    } else {
      input += c;
    }
  }
}

const float L1 = 1.5; //hip vertical
const float L2 = 5.7;  // femur
const float L3 = 7.9;   // tibia
const float L4=5; //hip horizontal

// === Compute IK for Front Right Leg ===
void computeRFLegIK(float footX, float footY, float footZ) {

  float servo1,servo2,servo3;
  float theta1,theta2,theta3;

  //1. get theta1
  theta1 = atan2(footY,footX)+acos(L4/sqrt(footX*footX+footY*footY));

  //2. calculate A
  float A=sqrt(footX*footX+footY*footY-L4*L4);

  //3. get theta3
  float k=(L2*L2+L3*L3-pow((L1-A),2)-footZ*footZ)/(2*L2*L3);
  k=constrain(k,-1,1);
  theta3=acos(k);

  //4. get theta2
  float t=(L2*L2+pow((L1-A),2)+footZ*footZ-L3*L3)/(2*L2*sqrt(pow((L1-A),2)+footZ*footZ));
  t=constrain(t,-1,1);
  theta2 = atan2((L1-A),footZ)-acos(t);

  //check angle values (in radians) IK computed 
  Serial.print("theta1: ");
  Serial.println(theta1);
  Serial.print("theta2: ");
  Serial.println(theta2);
  Serial.print("theta3: ");
  Serial.println(theta3);
  Serial.println("");

  //map() function uses integers only so convert to degrees 
  theta1 = theta1*180/M_PI;
  theta2 = theta2*180/M_PI;
  theta3 = theta3*180/M_PI;

  //conversion to values used by PCA9685 for RF LEG
  servo1=map(theta1, 0, -90,SERVO_0_3,SERVO_M90_3);
  servo2=map(theta2,-180 , -90, SERVO_M180_4,SERVO_M90_4);
  servo3=map(theta3, 90, 180, SERVO_90_5,SERVO_180_5);

  //check Servo driver values
  Serial.println(servo1);
  Serial.println(servo2);
  Serial.println(servo3);

  //move servos
  pwm.setPWM(3, 0,  servo1);
  pwm.setPWM(4, 0,  servo2);
  pwm.setPWM(5, 0,  servo3);


}


