  #include <Servo.h>
  Servo myservo;
  int duration;
  int distance;
  int trig = 13;
  int echo = 12;
  int servo = 8;

  int debug_flag = 1;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  myservo.attach(servo);
  
  pinMode(trig,OUTPUT); // setup my pin for output
  pinMode(echo,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  myservo.write(0);
  if (debug_flag == 1)
  {
    Serial.print("0 degrees \n");
  }
  
  delay(500);
  myservo.write(90);
  if (debug_flag == 1)
  {
    Serial.print("90 degrees \n");
  }

  delay(500);
  
  /*
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  duration = pulseIn(echo,HIGH);
  distance = (duration/2)/29.1; // in cm
  Serial.print(distance);
  Serial.print("\n");

  delay(500);
  */

}
