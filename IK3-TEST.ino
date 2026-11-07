#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



//The inverse kinematics equations are derived for the RIGHT FRONT leg.
//They can also be used for the other three legs, but only if each leg is
//considered from the right front leg's point of view.

//Note:
//The X axis (side-to-side movement) is mirrored between the left and right legs.
//This means that the same positive X value produces mirrored hip angles.
//For example, the same X coordinate will result in opposite sideways motion
//for the left and right legs.


//How the code works:
//get leg,x,y,z (input) -> Compute inverse kinematics -> Convert joint angles to PCA9685 values for the selected leg -> move the selected leg
//leg = 1 LEFT FRONT LEG
//leg = 2 RIGHT FRONT LEG
//leg = 3 LEFT BACK LEG
//leg = 4 RIGHT BACK LEG

//Before using this code, You need to know the PCA9685 servo values:
//all hip 0 -90
//all femur -90 -180
//all tibia 90 180
//If you don't know how to do it, open arduino file called 'SetServosPCA9685', then test it for one leg
//Once all servos are calibrated and every leg moves correctly, open arduino file called 'Trot'

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
  Serial.println("\nChoose one leg and enter target foot position (leg x y z) (x,y,z position in cm:)");
}

void loop() {
  static String input = "";

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        float footX, footY, footZ,leg;

        //expects "leg x y z"
        int space1 = input.indexOf(' ');
        int space2 = input.indexOf(' ', space1 + 1);
        int space3 = input.indexOf(' ', space2 + 1);
        if (space1 > 0 && space2 > space1 && space3 > space2) {

          leg = input.substring(0, space1).toInt();
          footX = input.substring(space1 + 1, space2).toFloat();
          footY = input.substring(space2 + 1, space3).toFloat();
          footZ = input.substring(space3 + 1).toFloat();

          computeIK(footX, footY, footZ, leg);

        } else {
          Serial.println("Invalid input. Use format: leg x y z");
        }

        Serial.println("\nChoose one leg and enter target foot position (leg x y z) (x,y,z position in cm:)");
        input = "";
      }
    } else {
      input += c;
    }
  }
}

const float L1 = 1.5; //hip distance vertical
const float L2 = 5.7;  // femur
const float L3 = 7.9;   // tibia
const float L4=5; //hip distance horizontal

// === Compute IK ===
void computeIK(float footX, float footY, float footZ, int leg) {

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




  if(leg==1){
    //conversion to values used by PCA9685 for LEFT FRONT LEG
    servo1=map(theta1, 0, -90,SERVO_0_0,SERVO_M90_0);
    servo2=map(theta2, -180, -90, SERVO_M180_1,SERVO_M90_1);
    servo3=map(theta3, 90, 180, SERVO_90_2,SERVO_180_2);
    Serial.println(servo1);
    Serial.println(servo2);
    Serial.println(servo3);

    //send them to servos
    pwm.setPWM(0, 0,  servo1);
    pwm.setPWM(1, 0,  servo2);
    pwm.setPWM(2, 0,  servo3);


  }else if(leg==2){  
    //conversion to values used by PCA9685 for RIGHT FRONT LEG
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

  }else if(leg==3){
    //conversion to values used by PCA9685 for LEFT BACK LEG
    servo1=map(theta1, 0, -90,SERVO_0_6,SERVO_M90_6);
    servo2=map(theta2,-180 , -90, SERVO_M180_7,SERVO_M90_7);
    servo3=map(theta3, 90, 180, SERVO_90_8,SERVO_180_8);

    //check Servo driver values
    Serial.println(servo1);
    Serial.println(servo2);
    Serial.println(servo3);

    //move servos
    pwm.setPWM(6, 0,  servo1);
    pwm.setPWM(7, 0,  servo2);
    pwm.setPWM(8, 0,  servo3);

  }

  else if(leg==4){
    //conversion to values used by PCA9685 for RIGHT BACK LEG
    servo1=map(theta1, 0, -90,SERVO_0_9,SERVO_M90_9);
    servo2=map(theta2,-180 , -90, SERVO_M180_10,SERVO_M90_10);
    servo3=map(theta3, 90, 180, SERVO_90_11,SERVO_180_11);

    //check Servo driver values
    Serial.println(servo1);
    Serial.println(servo2);
    Serial.println(servo3);

    //move servos
    pwm.setPWM(9, 0,  servo1);
    pwm.setPWM(10, 0,  servo2);
    pwm.setPWM(11, 0,  servo3);

  }}

