#define SERIAL_BAUD_RATE 115200

//Trigger - used to send signal out from sensor
//Echo - used to recieve signal bounced back from obstacle
const int triggerPin1 = 13;
const int echoPin1 = 3; //interrupt pin

//Values used for second sonar sensor
const int triggerPin2 = 11;
const int echoPin2 = 10;

//Duration - used to time how long the signal lasts once bounced back
//Distance - used to store distance value calculated using duration
long duration1;
int distance1;

long duration2;
int distance2;

void setup() {
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  //Attach sonarISR to interrupt to activate second sonar sensor once
  //first sensor has finished detection
  attachInterrupt(digitalPinToInterrupt(echoPin1), sonarISR, FALLING);
  Serial.begin(SERIAL_BAUD_RATE);

}

void loop() {
  //First sensor stuff
  //Writes an initial LOW value to pin to ensure it is not high
  digitalWrite(triggerPin1, LOW);
  delayMicroseconds(2);

  //Then a HIGH value is written to send initial signal
  //which is sent for 10 micro
  digitalWrite(triggerPin1, HIGH);
  delayMicroseconds(10);
  //Signal is then stopped by writing LOW to pin
  digitalWrite(triggerPin1, LOW);

  //Then record how long pulse is detected once bounced back from obstacle
  //and records time as long as pin is pulled HIGH
  duration1 = pulseIn(echoPin1, HIGH);
  //.000343 - meters, .0343 - cm, .343 - mm
  distance1 = duration1*.343/2;

  //Debug print statements
  //Serial.print("Distance(1): ");
  //Serial.println(distance1);
}

void sonarISR(){
  //ISR is called any time the echo pin of the first sonar is changed from HIGH to LOW
  //which means the first sensor has completed its reading
  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);

  //write high to send out pulse
  digitalWrite(triggerPin2, HIGH);
  //keep sending pulse for 10 micro
  delayMicroseconds(10);
  //stop sending pulse by bringing pin LOW
  digitalWrite(triggerPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH);
  //.000343 - meters, .0343 - cm, .343 - mm
  distance2 = duration2*.343/2;

  //Debug print statements
  //Serial.print("Distance(2): ");
  //Serial.println(distance2);
}
