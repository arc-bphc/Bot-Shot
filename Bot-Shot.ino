
// Bot-Shot
// version 2.0
// Author: Ebin Philip

// Changes:
// made changes in start state of motors to HIGH
// stop_robot is called by default in move_robot if servo_move is true


#include<Servo.h>
#include<SoftwareSerial.h>

// Servo objects for arm and gripper
Servo arm;
Servo gripper;

boolean servo_move=false;

int rx=7;
int tx=8;

// initial angles for arm and gripper servos
int arm_pos=170;
int gripper_pos=180;

// software serial is not used in this version of code

//SoftwareSerial mySerial= SoftwareSerial(rx,tx);

// setting motor output pins
int lmotor1=6;
int lmotor2=11;
int rmotor1=5;
int rmotor2=12;

// commands array will be storing the bluetooth input
byte commands[4]={0,0,0,0};

void bt_input(){
  
// to check serial buffer overflow in softwareserial library
// commented out since it is not used

//   if (mySerial.overflow()){
//     Serial.println("overflow occurred");
//     for(int i=0;i<4;++i)
//      commands[i]=0;
//     while(mySerial.available()){
//       mySerial.read();
//     }
//   }


// check if 4 bytes are available in the serial buffer
   if(Serial.available()>=4){

    // if the first byte read is not the first byte sent from the bluetooth skip it and read the next byte
    while(1){
      commands[0] = Serial.read();  //Direction
      // check if the first byte value corresponds to those sent by the app over bluetooth
      if(commands[0]==241 || commands[0]==242 || commands[0]==243)
        break;
    }  
    // read the next 3 bytes as well
    commands[1] = Serial.read();  //Speed
    commands[2] = Serial.read();  //Angle
 
    commands[3] = Serial.read(); 

    // send the read commands to the monitor
    Serial.print(commands[0]);
    Serial.print(" ");
    Serial.print(commands[1]);
    Serial.print(" ");
    Serial.print(commands[2]);
    Serial.print(" ");
    Serial.print(commands[3]);
    Serial.println(" ");  
    
    // use the input data to move the robot
    move_robot();
    
  }

  // check if button A is pressed on the app
  if(commands[3]==16 && !servo_move){
    servo_move=true;
    Serial.println("servo mode");
  }
  if(commands[3]==0 && servo_move){
    servo_move=false;
    Serial.println("exited servo mode");
  }
  
  // move_arm() moves both the arm and gripper servos
  move_arm();
  
}


// move_robot() is the master fuction for all wheel movements of the robot
// makes calls to forward(),back(),turn() and stop_robot() based on commands
void move_robot(){

  // execute only if not in servo control mode
  if(!servo_move){

  if(commands[0]==243 && commands[2]==89)
    stop_robot();

  else if(commands[0]==241 && commands[2]==89)
    forward(commands[1]);

  else if(commands[0]==242 && commands[2]==89)
    back(commands[1]);

  else if(commands[2]!=89)
    turn(commands[2]);
  
  else
    stop_robot();
  
    
  }

  // if servo_move is true, stop the wheels
  else
    stop_robot();
  
  
}




void move_arm(){

  if(!servo_move)
    return;

  gripper.attach(9);
  arm.attach(10);
  
  if(commands[1]!=0){
    
    if(commands[0]==241){
     arm_pos=arm_pos-1;
     if(arm_pos<70)
        arm_pos=70;
    }     
    else if(commands[0]==242){
     arm_pos=arm_pos+1;
     if(arm_pos>=170)
        arm_pos=170;
    }
    Serial.print("arm position: ");
    Serial.println(arm_pos);
  
  } 
  
  
  if(commands[2]>89){
   gripper_pos=gripper_pos-2;

   if(gripper_pos<70)
    gripper_pos=70;
   
   Serial.print("gripper: ");
   Serial.println(gripper_pos);
  }
  else if(commands[2]<89){
    gripper_pos=gripper_pos+2;
    
    if(gripper_pos>=180)
    gripper_pos=180;
    
    Serial.print("gripper: ");
    Serial.println(gripper_pos);
    
  }

  arm.write(arm_pos);
  gripper.write(gripper_pos);
  delay(10);
  

  //arm.detach();
  //gripper.detach();
  

}

void stop_robot(){
  digitalWrite(lmotor1,HIGH);
  digitalWrite(lmotor2,HIGH);
  digitalWrite(rmotor1,HIGH);
  digitalWrite(rmotor2,HIGH);

}

void forward(int wheel_speed){
  analogWrite(lmotor1,wheel_speed);
  digitalWrite(lmotor2,LOW);
  analogWrite(rmotor1,wheel_speed);
  digitalWrite(rmotor2,LOW);
}

void back(int wheel_speed){
  analogWrite(lmotor1,(255-wheel_speed));
  digitalWrite(lmotor2,HIGH);
  analogWrite(rmotor1,(255-wheel_speed));
  digitalWrite(rmotor2,HIGH);
}

void turn(int wheel_speed){
  if(commands[2]>89){

    //int val=map(commands[2],89,179,0,255);
    digitalWrite(lmotor1,HIGH);
    digitalWrite(lmotor2,LOW);
    digitalWrite(rmotor1,LOW);
    digitalWrite(rmotor2,HIGH);
  }

  else if(commands[2]<89){

    //int val=map(commands[2],89,0,0,255);
    digitalWrite(lmotor1,LOW);
    digitalWrite(lmotor2,HIGH);
    digitalWrite(rmotor1,HIGH);
    digitalWrite(rmotor2,LOW);
  }
}


void setup() {
  // put your setup code here, to run once:
  arm.attach(10);
  gripper.attach(9);
  gripper.write(180);
  arm.write(170);
  delay(1000);
  //gripper.detach();
  //arm.detach();
  
  pinMode(lmotor1,OUTPUT);
  pinMode(lmotor2,OUTPUT);
  pinMode(rmotor1,OUTPUT);
  pinMode(rmotor2,OUTPUT);

//  pinMode(rx,INPUT);
//  pinMode(tx,OUTPUT);
  
  Serial.begin(9600);
  //mySerial.begin(9600);

  digitalWrite(lmotor1,HIGH);
  digitalWrite(lmotor2,HIGH);
  digitalWrite(rmotor1,HIGH);
  digitalWrite(rmotor2,HIGH);

 delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
  bt_input();
  
}
