#include <NewPing.h>
#include<Servo.h>
#include<PID_v1.h>
/******************** Ultrasonic variables ********************/
  int TRIGGER_PIN = 12;  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  int ECHO_PIN = 11;  // Arduino pin tied to echo pin on the ultrasonic sensor.
  int MAX_DISTANCE = 40; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
  double Input, Output, Setpoint, rawInput, ServoOutput, smoothInput; 
/******************** Servo variables ********************/
  Servo myServo;  
  const int servoPin = 9; 
  int servoCenter = 90;
  
/******************** Smoothing variables ********************/
  int smooth[6];
  int smoothCounter = 0;  
    
/******************** PID variables ********************/
  float Kp = 2;        //Initial Proportional Gain
  float Ki = .5;       //Initial Integral Gain
  float Kd = 0;        //Intitial Derivative Gain .05
  double center = 15;
  double inputValue = 0; //for getting the abs value
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);           //Initialize PID object, which is in the class PID.
void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  myServo.attach(servoPin);     //Attach Servo
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(0,40); 
  
}
void loop() {
  
  /******************** Get ultrasonic data ********************/
  rawInput = readPosition(); //get ultrasonics distance data, Calls function readPosition()
  //Serial.print(rawInput);    // print input to terminal
  //Serial.print("\t");
  
 
  /******************** integrate averaging (smoothing for noise) ********************/
  
  for(int i = 0; i < 5; i++){
    smooth[i] = smooth[i + 1];
  }
  
  
  smooth[4] = rawInput;
  //smooth the data with a rolling average
  smoothInput = (smooth[0] + smooth[1] + smooth[2] + smooth[3] + smooth[4]) / 5; 
  
  inputValue = smoothInput - center;
  inputValue = abs(inputValue);
  Input = inputValue;  // use the smooth values for the PID input
  Serial.println(Input); 
  Serial.print(" \t");
  /******************** Use PID controls ********************/
  
  //Input = rawInput;
  Setpoint = 0;
  myPID.Compute();  //creates an "Output" value from the "Input" value using the PID variables
 
  
  if (smoothInput < center){
    ServoOutput = servoCenter - Output;  // offset in degrees - higher is clockwise
  }
  
  if (smoothInput > center){
    ServoOutput = servoCenter + Output;  // offset in degrees - higher is clockwise
  }
  
  myServo.write(ServoOutput);     // writes value of Output to servo
    
}
float readPosition() {
  delay(50);                                                            //Don't set too low or echos will run into eachother.      
  //const int pingPin = 7;
  long cm;                   // Wait 50ms between pings
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  cm = uS / (float)US_ROUNDTRIP_CM;
  if(cm > 40)     // 30 cm is the maximum position for the ball
  {cm=40;}  
  if(cm == 0){
    cm = 40; //done to get rid of null data
  }
  
  return cm;                                          //Returns distance value.
}
