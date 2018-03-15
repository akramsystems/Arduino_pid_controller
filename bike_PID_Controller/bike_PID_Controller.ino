///////////////////////////////////////////////////////////
//  PID_BIKE CONTROLLER
//  by Ali Akram
//  Notes:  Added code for new motor controller.
//
///////////////////////////////////////////////////////////
#include <PID_v1.h>






//PID STUFF
double Reference;  // will be the desired value
double Error;       // value of Reference - Output
double Input;       // Actual Roll angle of Bike (off balance or not)
double Output ;     // is the output from our PID Controller  so our new YAw angle 
//volatile float CO[3] = {0, 0, 0};             // Controller Output array for each part of the controller Kp, Ki and Kd
//volatile float E[3] = {0, 0, 0};              // Error for times t, t-1 and t-2
//volatile float C1 = 0;                        // Controller Output coefficient @ t-1
//volatile float C2 = 0;                        // Controller Output coefficient @ t-2
//volatile float E0 = 0;                        // Error coefficient @ t
//volatile float E1 = 0;                        // Error coefficient @ t-1
//volatile float E2 = 0;                        // Error coefficient @ t-2

//PID parameters
double Kp = 94.429, Ki = 0.025, Kd = 193.6;
int tolerance = 1;                // how accurate do we want to be in terms of our yaw angle

//STUFF FOR ACCELEROMETER
unsigned int rad;
float v = 5;       // velocity the bike is travelling in
int xAxis = A0;
int yAxis = A1;
int zAxis = A2;
int offsetX = 325; //roll angle
int offsetY = 355;
int offsetZ = 440;
int X_Value = 0;
int Y_Value = 0;
int Z_Value = 0;



//Serial Output Stuff
String Direction = "Not working";
String txt_output = "nothing";





//create PID instance
PID myPID(&Input, &Output, &Reference, Kp, Ki, Kd, DIRECT);


void setup()
{
  myPID.SetOutputLimits(-10000,10000);
  Serial.begin(9600);
  //Hardcode the brigdness value

  // we want roll to be 0 degrees
  Reference = 0;    
  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetSampleTime(200);        //determines time step for PID 
  
}


void loop()
{


  // Sets Input to Roll angle
  measure_angle();      

  // Gets Output from PID
  myPID.Compute();
  

  
  //gets the error from PID system

  
    Error = Reference - X_Value;

  

  //compute value output for PID and tells system to turn Right or Left depending on Error
  Change_Steering_Wheel_Value();
//  if (Error < -0.01) {
//    Direction = "Left";
//  } else if (Error > 0.01) {
//    Direction = "Right";
//  }
//  else {
//    Direction = "Nowhere";
//  }


//Serial Output for values
  Serial.print("Input:  ");
  Serial.print(Input);
  Serial.print("  ");
  Serial.print("Output: ");
  Serial.print(Output);
  Serial.print("  ");
  Serial.print("Error:  ");
  Serial.print(Error);
  Serial.print("  ");
  Serial.print("Turn: " + Direction);
  Serial.print("      ");
  Serial.print("  X:");
  Serial.print(X_Value);   
//  Serial.print("  Y:");
//  Serial.print(Y_Value); 
//  Serial.print("  Z:");
//  Serial.print(Z_Value);  
  Serial.println();
  
  delay(200);          // delay is  2ms
}



void Change_Steering_Wheel_Value() {

  // get yaw angle that we want
  //...
  
  if (Error < -1*tolerance) {
    Direction = "to the Left";
    
    
  } else if (Error > tolerance) {
    Direction = "to the Right";
  }
  else {
    Direction = "no where";
  }


 
  // tell motor to apply a certain amount of torque
  //...
  // Set new value for output:
  //...
  // test if system reacts to change in angle
}



void measure_angle() {
  // Roll angle is set to value from sensor
  X_Value = (analogRead(xAxis) - offsetX) / 2;
  Y_Value = (analogRead(yAxis) - offsetY) / 2;
  Z_Value = (analogRead(zAxis) - offsetZ) / 2;
  Input = X_Value;
}








