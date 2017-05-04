//Team: Shriya Bhatnagar, Austin Dodge, Michael Green
//Code for Salvador WALL-E Painting Robot

#include <math.h>
#define pi 3.14159265359
#define button 38

 double phiE = radians(0);  //fixed angle w.r.t. wrist

 //length of arm sections. May need to remeasure for better accuracy. Should total 32 cm.
 double r2 = 9; //from shoulder
 double r3 = 11; //from elbow
 double r4 = 12; //from wrist

 const int LED = 2; //pins for controlling robot arm
 const int M5Left = 3;
 const int M5Right = 4;
 const int M4Up = 5;
 const int M4Down = 6;
 const int M3Down = 7;
 const int M3Up = 8;
 const int M2Down = 9;
 const int M2Up = 10;
 const int M1Open = 11;
 const int M1Close = 12;

 int M1Fdbk, M2Fdbk, M3Fdbk, M4Fdbk, M5Fdbk;  //Feedback variables capture pot values
 double prevTheta1=1000, prevTheta2=1000, prevTheta3=1000, prevTheta4=1000;

 /*The following variables may vary on individual robot build. To fill these in,
1. arrange your robot arm in the zero position: lying flat and straight as seen in HW2,3. 
   Read the pot values at this position: these will be your M_zero values.
 
 2. To get the multiplication factor to convert degrees to analog pot values, find the following:
 *M2 pot value @ 60 degrees (maximum upward angle for this servo)
 *M3 pot value @ -90 degrees
 *M4 pot value @ 90 degrees
 *M5 pot value @ 90 or -90 degrees
 
 These values will then be used to obtain our DA values (degree to analog) with the following formula:
 
 DA = (M_Pot Value - M_Zero)/M_Degree Value
 
 where Pot and corresponding Degree values were found in step 2. The '_' correspond to servo numbers.*/

 int M1Op = 262;  //gripper open
 int M1Cl = 92; //gripper closed around brush holder
 int M2Zero = 311;  //pot value where theta4 = 0
 int M3Zero = 279;  //pot value where theta3 = 0
 int M4Zero = 412;  //pot value where theta2 = 0
 int M5Zero = 320;  //pot value where theta1 = 0

 double M2DA = -3.77; //multiplication factors to convert degrees to pot values
 double M3DA = -3.68;
 double M4DA = 3.84;
 double M5DA = -3.56;

//Set up pins//////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(button,INPUT);//initialize button

  pinMode(LED,OUTPUT); //classify pins as outputs
  pinMode(M5Left,OUTPUT); 
  pinMode(M5Right,OUTPUT); 
  pinMode(M4Up,OUTPUT); 
  pinMode(M4Down,OUTPUT); 
  pinMode(M3Down,OUTPUT); 
  pinMode(M3Up,OUTPUT);
  pinMode(M2Down,OUTPUT);
  pinMode(M2Up,OUTPUT);
  pinMode(M1Open,OUTPUT);
  pinMode(M1Close,OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  
  digitalWrite(LED,HIGH);  //relays are active low, so set all outputs high at start
  digitalWrite(M5Left,HIGH);
  digitalWrite(M5Right,HIGH);
  digitalWrite(M4Up,HIGH);
  digitalWrite(M4Down,HIGH);
  digitalWrite(M3Down,HIGH);
  digitalWrite(M3Up,HIGH);
  digitalWrite(M2Down,HIGH);
  digitalWrite(M2Up,HIGH);
  digitalWrite(M1Open,HIGH);
  digitalWrite(M1Close,HIGH);
}

//Motor Functions//////////////////////////////////////////////////////////////////
//all functions are active low
  void M1Servo(int t)  //open and close gripper
  {
    M1Fdbk = analogRead(A0);
    
    if (t == 0){             //open gripper
      while(M1Fdbk < M1Op){
        digitalWrite(M1Close, HIGH);
        digitalWrite(M1Open, LOW);
        M1Fdbk = analogRead(A0);
        }
      digitalWrite(M1Open, HIGH);  
    }
    else if(t == 1){       //close gripper
      while(M1Fdbk > M1Cl){
        digitalWrite(M1Close, LOW);
        digitalWrite(M1Open, HIGH);
        M1Fdbk = analogRead(A0);
      }
     digitalWrite(M1Close, HIGH);
    }
    else{                     //stay still if already at requested angle
      digitalWrite(M1Close, HIGH);
      digitalWrite(M1Open, HIGH);
    } 
  }

void M2Servo(double t){  //Move wrist up for specified angle t in degrees

    int tAna = (t*M2DA) + M2Zero; //convert angle -> analog value
    M2Fdbk = analogRead(A1);  //Read current pot value for M2
   
    
    if (tAna < M2Fdbk){             //from vertical (when position = L), go left
      while(tAna < M2Fdbk){
        digitalWrite(M2Down, HIGH);
        digitalWrite(M2Up, LOW);
        M2Fdbk = analogRead(A1);
        }
      digitalWrite(M2Up, HIGH);  
    }
    else if(tAna > M2Fdbk){       //from position = L, go right
      while(tAna > M2Fdbk){
        digitalWrite(M2Down, LOW);
        digitalWrite(M2Up, HIGH);
        M2Fdbk = analogRead(A1);
      }
     digitalWrite(M2Down, HIGH);
    }
    else{                     //stay still if already at requested angle
      digitalWrite(M2Down, HIGH);
      digitalWrite(M2Up, HIGH);
    } 
  }

void M3Servo(double t)  //Move wrist up for specified angle t in degrees
  {
    int tAna = (t*M3DA) + M3Zero; //convert angle -> analog value
    M3Fdbk = analogRead(A2);  //Read current pot value for M3
    
    if (tAna < M3Fdbk){             //from vertical (when position = L), go left
      while(tAna < M3Fdbk){
        digitalWrite(M3Down, HIGH);
        digitalWrite(M3Up, LOW);
        M3Fdbk = analogRead(A2);
        }
      digitalWrite(M3Up, HIGH);  
    }
    
    else if(tAna > M3Fdbk){       //from position = L, go right
      while(tAna > M3Fdbk){
        digitalWrite(M3Down, LOW);
        digitalWrite(M3Up, HIGH);
        M3Fdbk = analogRead(A2);
      }
     digitalWrite(M3Down, HIGH);
    }

    else{                     //stay still if already at requested angle
      digitalWrite(M3Down, HIGH);
      digitalWrite(M3Up, HIGH);
    } 
  }

void M4Servo(double t)  //Move wrist up for specified angle t in degrees
  {
    int tAna = (t*M4DA) + M4Zero; //convert angle -> analog value
    M4Fdbk = analogRead(A3);  //Read current pot value for M4
    
    if (tAna < M4Fdbk){             //from vertical (when position = L), go left
      while(tAna < M4Fdbk){
        digitalWrite(M4Down, HIGH);
        digitalWrite(M4Up, LOW);
        M4Fdbk = analogRead(A3);
        }
      digitalWrite(M4Up, HIGH);  
    }
    else if(tAna > M4Fdbk){       //from position = L, go right
      while(tAna > M4Fdbk){
        digitalWrite(M4Down, LOW);
        digitalWrite(M4Up, HIGH);
        M4Fdbk = analogRead(A3);
      }
     digitalWrite(M4Down, HIGH);
    }
    else{                     //stay still if already at requested angle
      digitalWrite(M4Down, HIGH);
      digitalWrite(M4Up, HIGH);
    } 
  }

 void M5Servo(double t)  //Move wrist up for specified angle t in degrees
  {
    int tAna = (t*M5DA) + M5Zero; //convert angle -> analog value
    M5Fdbk = analogRead(A4);  //Read current pot value for M4
    
    if (tAna > M5Fdbk){             //from vertical (when position = L), go left
      while(tAna > M5Fdbk){
        digitalWrite(M5Right, HIGH);
        digitalWrite(M5Left, LOW);
        M5Fdbk = analogRead(A4);
        }
      digitalWrite(M5Left, HIGH);  
    }
    
    else if(tAna < M5Fdbk){       //from position = L, go right
      while(tAna < M5Fdbk){
        digitalWrite(M5Right, LOW);
        digitalWrite(M5Left, HIGH);
        M5Fdbk = analogRead(A4); 
      }
     digitalWrite(M5Right, HIGH);
    }

    else{                     //stay still if already at requested angle
      digitalWrite(M5Right, HIGH);
      digitalWrite(M5Left, HIGH);
    } 
  }

void Blink() //Blink LED to signal end of set
{
  for(int i = 0; i < 4; i++)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}

//Inverse Kinematics////////////////////////////////////////////////////////////////////

/*The following function will take in XYZ coordinates, which represent a point for the gripper to reach in cm.
Point [0,0,0] should be at the center of the robot base (under servo M4). XYZ plane is shown by right hand rule. 
Your fingers point in the positive directions (thumb in z direction, forefinger in x direction). Arm faces x direction.

To find IK formulas I referenced example on pg 4 of 
https://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter4.pdf */

void Move2(double x, double y, double z){   //calculate angles from xyz coordinates via inverse kinematics. xyz in cm.
  
  double theta1 = atan2(y,x)*57.296;  //base angle in degrees

  double xW = x - r4*cos(phiE);   //wrist coordinates
  double zW = (z - r4*sin(phiE))-0; //the 1.5 cm factor allows brush to reach paper 

  double H = sqrt((xW*xW) + (zW*zW));
  double alpha = atan2(zW,xW);
  double gamma = acos(((r2*r2) + (H*H) - (r3*r3))/(2*r2*H));
  double theta2 = (alpha + gamma)*57.296;   //shoulder angle in degrees

  double beta = acos(((r3*r3) + (r2*r2) - (H*H))/(2*r2*r3));
  double theta3 = (beta - pi)*57.296;   //elbow angle in degrees. For angles below 0 degrees.

  double theta4 = (phiE*57.296) - theta2 - theta3;// - theta2 - theta3;  //wrist angle in degrees
  
  if (theta1 != prevTheta1){  //if loops prevent the arm from stuttering.
    M5Servo(theta1);
    prevTheta1 = theta1;
  }

  if (theta2 != prevTheta2){
    M4Servo(theta2);
    prevTheta2 = theta2;
  }

  if (theta4 != prevTheta4){
    M2Servo(theta4);
    prevTheta4 = theta4;
  }
    if (theta3 != prevTheta3){
    M3Servo(theta3);
    prevTheta3 = theta3;
  }

  
}

//Main Program////////////////////////////////////////////////////////////////////
void loop() {

  Move2(23, 12, 15); //get in starting position to get brush
  M1Servo(0);   //open gripper

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

  M1Servo(1); //close gripper
  M3Servo(-40); //lift brush to prevent touching paper too early

  for (double c = 28; c >= 22; c = c-2){
    for(double b = 8.5; b >= -12; b = b-0.25){  //paint 1st layer of the sky       
      Move2(c, b, 6); 
       }
    }
    M3Servo(-40); //lift brush to prevent touching paper 

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);         //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }
  
  M1Servo(1); //close gripper
  M3Servo(0); //lift brush to prevent touching paper too early
  for (double c1 = 28; c1 >= 22; c1 = c1-2){
    for(double b1 = -12; b1 <= 8.5; b1 = b1+0.25){  //paint 2nd layer of sky, go in opposite direction from the first       
      Move2(c1, b1, 6); 
       }
    }
    M3Servo(-40); //lift brush to prevent touching paper

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);       //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }
  
  M1Servo(1); //close gripper
  M3Servo(0); //lift brush to prevent touching paper too early

  for (double c2 = 22; c2>=20; c2=c2-1){
    for(double b2 = 8.5; b2 >= -12; b2=b2-0.25){  //paint 1st layer of ground
      Move2(c2, b2, 5.5); 
    }
  }
    M3Servo(-40); //lift brush to prevent touching paper

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);     //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

  M1Servo(1); //close gripper
  M3Servo(0); //lift brush to prevent touching paper too early

  for (double c3 = 22; c3 >= 20; c3 = c3-1){
    for(double b3 = -12; b3 <= 8.5; b3 = b3+0.25){  //paint 2nd layer of ground, in opposite direction of 1st
      Move2(c3, b3, 5.5); 
    }
  }
    M3Servo(-40); //lift brush to prevent touching paper

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);       //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

   M1Servo(1); //close gripper
   M3Servo(-40);  //lift brush to prevent touching paper too early
   
   for(double a = 22; a <= 30; a=a+0.25){  //draw middle skyscraper, tallest height
    Move2(a, 0, 6); 
  }
  M4Servo(90);  //lift brush to prevent touching paper
  M3Servo(0);

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);         //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

   M1Servo(1); //close gripper
   M3Servo(-40);  //lift brush to prevent touching paper
   
   for(double a = 22; a <= 26; a=a+0.25){  //draw far left building, medium height
    Move2(a, 4, 6);
  }
  M4Servo(90);  //lift brush to prevent touching paper
  M3Servo(0);

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);       //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

  M1Servo(1); //close gripper
   M3Servo(-40);  //lift brush to prevent touching paper

   for(double a = 22; a <= 23; a=a+0.25){  //center left building. Shortest height
    Move2(a, 2, 6);
  }
  M4Servo(90);  //lift brush to prevent touching paper
  M3Servo(0);

  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);         //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

   M1Servo(1); //close gripper
   M3Servo(-40);  //lift brush to prevent touching paper
   
   for(double a = 22; a <= 26; a=a+0.25){  //center right building. Medium height
    Move2(a, -4, 6); 
  }
  
  Move2(23, -12, 15);  //move to opposite end to get rid of old brush
  M1Servo(0);       //drop brush
  Move2(23, 12, 15); //back in starting position to get next brush

  while(digitalRead(button) == 0){  //Load brush. Wait for button press to continue
    ;
  }

   M1Servo(1); //close gripper
   M3Servo(-40);  //lift brush to prevent touching paper
   
   for(double a = 22; a <= 24; a=a+0.25){  //far right building. Short height.
    Move2(a, -7, 6); 
  }

  M4Servo(90);  //lift brush to prevent touching paper
  M3Servo(0);
  
  Blink();  //LED blinks to signal finished painting
    
}
