  int duration;
  int distance;
  int trig = 13;
  int echo = 12;
  int motor = 8;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  
  pinMode(trig,OUTPUT); // setup my pin for output
  pinMode(echo,INPUT);
  pinMode(motor,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motor,LOW);

  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  duration = pulseIn(echo,HIGH);
  distance = (duration/2)/29.1; // in cm
  Serial.print(distance);
  Serial.print("\n");

  if (distance < 8)
  {
  digitalWrite(motor,HIGH);
  }

  delay(500);

}
