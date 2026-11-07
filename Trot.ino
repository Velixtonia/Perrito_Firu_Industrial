#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*code overview:
  Features:
  - Trot forward/backward
  - Sideways movement
  - Turning
  - Smooth interpolation between poses
  - Optional ultrasonic recovery system

   Before running:
   1. Calibrate every servo using SetServosPCA9685.
   2. Replace the SERVO_* values below with your own calibration values.

 */


//Ultrasonic distance sensor is completely optional, if you want to use it uncomment distance() and CheckAndRecover()

//The inverse kinematics equations are derived for the RIGHT FRONT leg.
//They can also be used for the other three legs, but only if each leg is
//considered from the right front leg's point of view.

//Note:
//The X axis (side-to-side movement) is mirrored between the left and right legs.
//This means that the same positive X value produces mirrored hip angles.
//For example, the same X coordinate will result in opposite sideways motion
//for the left and right legs.


//Before using this code, You need to know the PCA9685 servo values:
//all hip 0 -90
//all femur -90 -180
//all tibia 90 180
//If you don't know how to do it, open arduino file called 'SetServosPCA9685', and then change all of the PCA9685 SERVO VALUES


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

// True when the robot is standing in its default position.
bool StartPos = false; 

const float L1 = 1.5; //hip vertical
const float L2 = 5.7;  // femur
const float L3 = 7.9;   // tibia

const float L4=5; //hip horizontal
int servo1,servo2,servo3;

//You might have to experiment a bit and change these values for your robot to work
float StartX=4;
float StartY=-10;
float StartZ=-1;

//Distance sensor pins - optional
//const int echo_pin = 11;
//const int trig_pin = 10;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
  //  pinMode(echo_pin, INPUT);
  // pinMode(trig_pin, OUTPUT);
  //digitalWrite(trig_pin, LOW);


    computeIK(StartX,StartY,StartZ,servo1,servo2,servo3,1);
    pwm.setPWM(0,0, servo1);
    pwm.setPWM(1, 0, servo2);  
    pwm.setPWM(2,0, servo3);
    computeIK(StartX,StartY,StartZ,servo1,servo2,servo3,2);
    pwm.setPWM(3,0, servo1);
    pwm.setPWM(4, 0, servo2);  
    pwm.setPWM(5,0, servo3);
    computeIK(StartX,StartY,StartZ,servo1,servo2,servo3,3);
    pwm.setPWM(6,0, servo1);
    pwm.setPWM(7, 0, servo2);  
    pwm.setPWM(8,0, servo3);
    computeIK(StartX,StartY,StartZ,servo1,servo2,servo3,4);
    pwm.setPWM(9,0, servo1);
    pwm.setPWM(10, 0, servo2);  
    pwm.setPWM(11,0, servo3);

  StartPos = true;
  delay(2000);

}


void loop() {

TrotForward(10,200);
//delay(2000);
//TrotBackward(10,50);
//delay(2000);
//TrotRight(5,100);
//delay(2000);
//TrotLeft(5,20);
//delay(2000);
//TurnLeft(10,50);
//delay(2000);
//TurnRight(10,50);

}




//----------------------------------------------------
// Gait functions
//----------------------------------------------------
// t      = delay multiplier controlling gait speed
// cycles = number of gait cycles to execute
//----------------------------------------------------

void TrotForward(int t,int cycles){  

  int X=3;
  int BX=4;
  int floorY=-12;
  int ST=4; //number of steps of the interpolation function

  float AZ=-3;
  float BY=-8;
  float BZ=2;
  float CZ=-1;
  float DZ=-1;
  //You might have to experiment a bit and change these values to make your robot move

    float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {X,floorY,CZ},
      {X,floorY,AZ},
      {X,floorY,AZ},
      {X,floorY,CZ}
    };

  if(StartPos==true){

    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    /*
    if(CheckAndRecover(NewPos,abs(StartY)-5)==true){
      return;
    }*/
    StartPos=false;

  }
  if(StartPos==false){

    for(int i=0;i<cycles;i++){

      float NewPos1[4][3]={
        {X,floorY,DZ},
        {BX,BY,BZ},
        {BX,BY,BZ},
        {X,floorY,DZ}
      };
      interpolation(NewPos,NewPos1,ST); 
      delay(t);
      /*
      if(CheckAndRecover(NewPos1,abs(floorY)-5)==true){
        return;
      }*/

      float NewPos2[4][3]={
        {X,floorY,AZ},
        {X,floorY,CZ},
        {X,floorY,CZ},
        {X,floorY,AZ}
      };
      interpolation(NewPos1,NewPos2,ST);
      delay(t*10);
      /*
      if(CheckAndRecover(NewPos2,abs(floorY)-5)==true){
        return;
      }*/

      float NewPos3[4][3] = {
        {BX, BY, BZ},
        {X, floorY, DZ},
        {X, floorY, DZ},
        {BX, BY, BZ},

      };
      interpolation(NewPos2,NewPos3,ST);
      delay(t);
      /*
      if(CheckAndRecover(NewPos3,abs(floorY)-5)==true){
        return;
      }*/

      float NewPos4[4][3] = {
        {X, floorY, CZ},
        {X, floorY, AZ},
        {X, floorY, AZ},
        {X, floorY, CZ},
      };
      interpolation(NewPos3, NewPos4, ST);
      delay(t*10);
      /*
      if(CheckAndRecover(NewPos4,abs(floorY)-5)==true){
        return;
      }      */
    }
    
      /*
      if(CheckAndRecover(NewPos5,abs(StartY)-5)==true){
         return;
      }*/
  ReturnToStartPosition(NewPos);
  }
}

void TrotBackward(int t,int cycles){

  float X=4;
  int BX=4;
  int floorY=-12;
  int ST=6; //number of steps of the interpolation function
  float AZ=1;
  float BY=-8;
  float BZ=-3;
  float CZ=-2;
  float DZ=-1;
  //You might have to experiment a bit and change these values to make your robot move

    float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {X,floorY,CZ},
      {X,floorY,AZ},
      {X,floorY,AZ},
      {X,floorY,CZ}
    };

  if(StartPos==true){

    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    /*
    if(CheckAndRecover(NewPos,abs(StartY)-5)==true){
      return;
    }*/
    StartPos=false;

  } 
  if(StartPos==false){

    for(int i=0;i<cycles;i++){

    float NewPos1[4][3]={
      {X,floorY,DZ},
      {BX,BY,BZ},
      {BX,BY,BZ},
      {X,floorY,DZ}
    };
    interpolation(NewPos,NewPos1,ST); 
    delay(t);
    /*
    if(CheckAndRecover(NewPos1,abs(StartY)-5)==true){
      return;
    }
    */
    float NewPos2[4][3]={
      {X,floorY,AZ},
      {X,floorY,CZ},
      {X,floorY,CZ},
      {X,floorY,AZ}
    };
    interpolation(NewPos1,NewPos2,ST);
    delay(t*10);

    /*
    if(CheckAndRecover(NewPos2,abs(StartY)-5)==true){
      return;
    }
    */
    float NewPos3[4][3] = {
      {BX, BY, BZ},
      {X, floorY, DZ},
      {X, floorY, DZ},
      {BX, BY, BZ},

    };
    interpolation(NewPos2,NewPos3,ST);// LF c to a RF b to modified c
    delay(t);

    /*
    if(CheckAndRecover(NewPos3,abs(StartY)-5)==true){
      return;
    }
    */

    float NewPos4[4][3] = {
      {X, floorY, CZ},
      {X, floorY, AZ},
      {X, floorY, AZ},
      {X, floorY, CZ},
    };
    interpolation(NewPos3, NewPos4, ST);
    delay(t*10);

    /*
    if(CheckAndRecover(NewPos4,abs(StartY)-5)==true){
      return;
    }*/
  }      


        /*
        if(CheckAndRecover(NewPos5,abs(StartY)-5)==true){
          return;
        }*/
  ReturnToStartPosition(NewPos);
  }
}

void TrotRight(int t,int cycles){  
  //recover function not implemented
  int FrontZ=-1;
  int BackZ=-3;
  int floorY=-12;
  int ST=6; //number of steps of the interpolation function
  float AX=5;
  float BY=-7;
  float BXR=6;
  float BXL=2;
  float CX=3;
  float DX=4;

  //You might have to experiment a bit and change these values to make your robot move
  float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {CX,floorY,FrontZ},
      {CX,floorY,FrontZ},
      {AX,floorY,BackZ},
      {AX,floorY,BackZ}
    };

  if(StartPos==true){

    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    StartPos=false;

  }  
  if(StartPos==false){
        for(int i=0;i<cycles;i++){

  float NewPos1[4][3]={
    {DX,floorY,FrontZ},
    {BXR,BY,FrontZ},
    {BXL,BY,BackZ},
    {DX,floorY,BackZ}
  };
  interpolation(NewPos,NewPos1,ST); 
  delay(t);

  float NewPos2[4][3]={
    {AX,floorY,FrontZ},
    {AX,floorY,FrontZ},
    {CX,floorY,BackZ},
    {CX,floorY,BackZ}
  };
  interpolation(NewPos1,NewPos2,ST);
  delay(t*10);

  float NewPos3[4][3] = {
    {BXL, BY, FrontZ},
    {DX, floorY, FrontZ},
    {DX, floorY, BackZ},
    {BXR, BY,BackZ},

  };
  interpolation(NewPos2,NewPos3,ST);
  delay(t);

  float NewPos4[4][3] = {
    {CX, floorY, FrontZ},
    {CX, floorY, FrontZ},
    {AX, floorY, BackZ},
    {AX, floorY, BackZ},
  };
  interpolation(NewPos3, NewPos4, ST);
  delay(t*10);
  }

  ReturnToStartPosition(NewPos);
  }
}

void TrotLeft(int t,int cycles){  
  //recover function not implemented
  int FrontZ=1;
  int BackZ=-3;
  int floorY=-12;
  int ST=6; //number of steps of the interpolation function
  float AX=3;
  float BY=-7;
  float BXR=3;
  float BXL=6;
  float CX=5;
  float DX=4;

  //You might have to experiment a bit and change these values to make your robot move

    float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {CX,floorY,FrontZ},
      {CX,floorY,FrontZ},
      {AX,floorY,BackZ},
      {AX,floorY,BackZ}
    };

  if(StartPos==true){
    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    StartPos=false;
  }  
  if(StartPos==false){
        for(int i=0;i<cycles;i++){

  float NewPos1[4][3]={
    {DX,floorY,FrontZ},
    {BXR,BY,FrontZ},
    {BXL,BY,BackZ},
    {DX,floorY,BackZ}
  };
  interpolation(NewPos,NewPos1,ST); 
  delay(t);

  float NewPos2[4][3]={
    {AX,floorY,FrontZ},
    {AX,floorY,FrontZ},
    {CX,floorY,BackZ},
    {CX,floorY,BackZ}
  };
  interpolation(NewPos1,NewPos2,ST);
  delay(t*10);

  float NewPos3[4][3] = {
    {BXL, BY, FrontZ},
    {DX, floorY, FrontZ},
    {DX, floorY, BackZ},
    {BXR, BY,BackZ},

  };
  interpolation(NewPos2,NewPos3,ST);
  delay(t);

  float NewPos4[4][3] = {
    {CX, floorY, FrontZ},
    {CX, floorY, FrontZ},
    {AX, floorY, BackZ},
    {AX, floorY, BackZ},
  };
  interpolation(NewPos3, NewPos4, ST);
  delay(t*10);
  }
  ReturnToStartPosition(NewPos);

  }
}

void TurnRight(int t,int cycles){  

  float FrontZ=1;
  float BackZ=-3;
  int floorY=-11;
  int ST=6; //number of steps of the interpolation function
  float AX=2;
  float BY=-7;
  float BX2=5;
  float BX1=1;
  float CX=4;
  float DX=3;
  //You might have to experiment a bit and change these values to make your robot move
    float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {AX,floorY,FrontZ},
      {AX,floorY,FrontZ},
      {AX,floorY,BackZ},
      {AX,floorY,BackZ}
    };


  if(StartPos==true){

    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    StartPos=false;

  }  
  if(StartPos==false){
    for(int i=0;i<cycles;i++){

      float NewPos1[4][3]={
        {DX,floorY,FrontZ},
        {BX2,BY,FrontZ},
        {BX2,BY,BackZ},
        {DX,floorY,BackZ}
      };
      interpolation(NewPos,NewPos1,ST); 
      delay(t);

      float NewPos2[4][3]={
        {CX,floorY,FrontZ},
        {CX,floorY,FrontZ},
        {CX,floorY,BackZ},
        {CX,floorY,BackZ}
      };
      interpolation(NewPos1,NewPos2,ST);
      delay(t*10);

      float NewPos3[4][3] = {
        {BX1, BY, FrontZ},
        {DX, floorY, FrontZ},
        {DX, floorY, BackZ},
        {BX1, BY,BackZ},

      };
      interpolation(NewPos2,NewPos3,ST);
      delay(t);


      float NewPos4[4][3] = {
        {AX, floorY, FrontZ},
        {AX, floorY, FrontZ},
        {AX, floorY, BackZ},
        {AX, floorY, BackZ},
      };
      interpolation(NewPos3, NewPos4, ST);
      delay(t*10);
  }

  ReturnToStartPosition(NewPos);
}
}

void TurnLeft(int t,int cycles){  

  float FrontZ=-1;
  float BackZ=-3;
  int floorY=-11;
  int ST=6; //number of steps of the interpolation function
  float AX=4;
  float BY=-7;
  float BX1=5;
  float BX2=1;
  float CX=2;
  float DX=3;


  //You might have to experiment a bit and change these values to make your robot move

    float OldPos[4][3]={
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ},
      {StartX,StartY,StartZ}

    };
    float NewPos[4][3]={
      {AX,floorY,FrontZ},
      {AX,floorY,FrontZ},
      {AX,floorY,BackZ},
      {AX,floorY,BackZ}
    };
  if(StartPos==true){

    interpolation(OldPos,NewPos,ST); 
    delay(t*100);
    StartPos=false;

  }  
  if(StartPos==false){
    for(int i=0;i<cycles;i++){

      float NewPos1[4][3]={
        {DX,floorY,FrontZ},
        {BX2,BY,FrontZ},
        {BX2,BY,BackZ},
        {DX,floorY,BackZ}
      };
      interpolation(NewPos,NewPos1,ST); 
      delay(t);

      float NewPos2[4][3]={
        {CX,floorY,FrontZ},
        {CX,floorY,FrontZ},
        {CX,floorY,BackZ},
        {CX,floorY,BackZ}
      };
      interpolation(NewPos1,NewPos2,ST);// LF c to a RF b to modified c
      delay(t*10);
      float NewPos3[4][3] = {
        {BX1, BY, FrontZ},
        {DX, floorY, FrontZ},
        {DX, floorY, BackZ},
        {BX1, BY,BackZ},

      };
      interpolation(NewPos2,NewPos3,ST);// LF c to a RF b to modified c
      delay(t);

      float NewPos4[4][3] = {
        {AX, floorY, FrontZ},
        {AX, floorY, FrontZ},
        {AX, floorY, BackZ},
        {AX, floorY, BackZ},
      };
      interpolation(NewPos3, NewPos4, ST);
      delay(t*10);
  }
  ReturnToStartPosition(NewPos);
}
}

void computeIK(float footX, float footY, float footZ, int &servo1, int &servo2, int &servo3, int leg) {
  //Converts a desired foot position (X, Y, Z)
  //into PCA9685 pulse values for the selected leg.
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

  //map() function uses integers only so convert to degrees 
  theta1=theta1*180/M_PI;
  theta2=theta2*180/M_PI;
  theta3=theta3*180/M_PI;

  if(leg==1){
    //conversion to values used by PCA9685 for LEFT FRONT LEG
    servo1=map(theta1, 0, -90,SERVO_0_0,SERVO_M90_0);
    servo2=map(theta2, -180, -90, SERVO_M180_1,SERVO_M90_1);
    servo3=map(theta3, 90, 180, SERVO_90_2,SERVO_180_2);
    //Serial.println(servo1);
    //Serial.println(servo2);
    //Serial.println(servo3);
  }else if(leg==2){  
    //conversion to values used by PCA9685 for RIGHT FRONT LEG
    servo1=map(theta1, 0, -90,SERVO_0_3,SERVO_M90_3);
    servo2=map(theta2,-180 , -90, SERVO_M180_4,SERVO_M90_4);
    servo3=map(theta3, 90, 180, SERVO_90_5,SERVO_180_5);

    //check Servo driver values
    //Serial.println(servo1);
    //Serial.println(servo2);
    //Serial.println(servo3);
  }else if(leg==3){
    //conversion to values used by PCA9685 for LEFT BACK LEG
    servo1=map(theta1, 0, -90,SERVO_0_6,SERVO_M90_6);
    servo2=map(theta2,-180 , -90, SERVO_M180_7,SERVO_M90_7);
    servo3=map(theta3, 90, 180, SERVO_90_8,SERVO_180_8);

    //check Servo driver values
    //Serial.println(servo1);
    //Serial.println(servo2);
    //Serial.println(servo3);
  }

  else if(leg==4){
    //conversion to values used by PCA9685 for RIGHT BACK LEG
    servo1=map(theta1, 0, -90,SERVO_0_9,SERVO_M90_9);
    servo2=map(theta2,-180 , -90, SERVO_M180_10,SERVO_M90_10);
    servo3=map(theta3, 90, 180, SERVO_90_11,SERVO_180_11);

    //check Servo driver values
    //Serial.println(servo1);
    //Serial.println(servo2);
    //Serial.println(servo3);
  }}

void interpolation(float OldPos[4][3], float NewPos[4][3], int steps) {
  int servo1, servo2, servo3;
  float Increment[4][3];
  float CurrentPos[4][3]; // Temporary array to track movement

  //Calculate increments and set the starting position
  for(int i = 0; i < 4; i++) {
    for(int m = 0; m < 3; m++) {
      Increment[i][m] = (NewPos[i][m] - OldPos[i][m]) / steps;
      CurrentPos[i][m] = OldPos[i][m]; // Copy OldPos to start
    }
  }

  //Move servos step-by-step
  for(int i = 0; i < steps; i++) {
    
    for(int m = 0; m < 4; m++) {
      // Compute IK using the temporary CurrentPos
      computeIK(CurrentPos[m][0], CurrentPos[m][1], CurrentPos[m][2], servo1, servo2, servo3, m+1);
      
      pwm.setPWM(3*m,0,servo1);
      pwm.setPWM(3*m+1,0,servo2);  
      pwm.setPWM(3*m+2,0,servo3);

      // Update CurrentPos for the next step
      CurrentPos[m][0] += Increment[m][0];
      CurrentPos[m][1] += Increment[m][1];
      CurrentPos[m][2] += Increment[m][2];
    }
    
    //Give the physical servos time to reach this step
    //Adjust this number (e.g., 5 to 20) to change the overall speed of the robot
    delay(10); 
  }
}

//Completely optional
//if you want to use it, uncomment these:
/*
float Distance(){
  //measure distance to the ground
  float timing = 0.0;
  float distance = 0.0;
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.0343) / 2;

  return distance;
}

bool CheckAndRecover(float OldPos[4][3],float treshold){
  //You might have to experiment a bit and change these values for your robot to work


    //check distance to the ground and recover
    float DefPos[4][3]={
    {StartX,StartY,StartZ},
    {StartX,StartY,StartZ},
    {StartX,StartY,StartZ},
    {StartX,StartY,StartZ}
  };

    float DefPos1[4][3]={
    {StartX,StartY,StartZ},
    {StartX,-8,-3},
    {StartX,-8,-3},
    {StartX,StartY,StartZ}
  };
    float DefPos2[4][3]={
    {StartX,StartY-1,StartZ},
    {StartX,StartY-1,-3},
    {StartX,StartY-1,-3},
    {StartX,StartY-1,StartZ}
  };
     float DefPos3[4][3]={
    {StartX,-8,-3},
    {StartX,StartY,-3},
    {StartX,StartY,-3},
    {StartX,-8,-3}
  }; 

  float DefPos4[4][3]={
    {StartX,StartY,-3},
    {StartX,StartY,-3},
    {StartX,StartY,-3},
    {StartX,StartY,-3}
  }; 

  if(Distance()<=treshold){
    interpolation(OldPos,DefPos,10); 
    delay(1000);
    interpolation(DefPos,DefPos1,10); 
    delay(1000);
    interpolation(DefPos1,DefPos2,10); 
    delay(1000);
    interpolation(DefPos2,DefPos3,10); 
    delay(1000);
    interpolation(DefPos3,DefPos4,10); 
    Serial.println(Distance());
    delay(1000);
    ReturnToStartPosition(DefPos4);

    return true;
  }else{
    return false;
  }

}
*/
void ReturnToStartPosition(float OldPos[4][3]){
  //Smoothly moves all four feet back to the default standing position.
        float NewPos[4][3] = {
          {StartX, StartY, StartZ},
          {StartX, StartY, StartZ},
          {StartX, StartY, StartZ},
          {StartX, StartY, StartZ},
        };    
        interpolation(OldPos, NewPos, 10);
      StartPos=true;
}