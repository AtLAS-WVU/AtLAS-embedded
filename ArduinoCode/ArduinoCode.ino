#include <TimerOne.h>

// There are only 6 usable channels, but for some reason, the receiver outputs 8 PPM
// channels, where the last 2 just never change.
#define NUM_CHANNELS 8

#define NUM_INPUT_CHANNELS 4
// Pin for PPM input. Must be either pin 1 or pin 2, because those are the only
// pins that support interrupts.
#define PPM_INPUT 2

// Total width of one PPM frame, in microseconds. This was determined by the receiver.
#define FRAME_WIDTH 20000
// Pin on which to output a PPM signal
#define PPM_OUTPUT 8

#define SERIAL_BAUD_RATE 115200

// Channels of remote controller inputs and outputs
// Note: This program uses 0-based indexing, but the remote control, and the LibrePilot
// GCS software use 1-based index. Therefore, channel 0 here corresponds to channel 1 of the
// remote control and GCS, and so on.
#define THROTTLE_CH 2
#define ROLL_CH 0
#define PITCH_CH 1
#define YAW_CH 3
// This is the channel for the manual control switch
#define MANUAL_CONTROL_CH 5

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

// Constantly updates to store the current values of the ppm inputs
volatile unsigned int ppmInput[NUM_CHANNELS] = {0};
// Change this array at any time to change the PPM output
volatile unsigned int ppmOutput[NUM_CHANNELS] = {0};

void setup() {

  //Added pin mappings for sonar sensors
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  //Attach sonarISR to interrupt to activate second sonar sensor once
  //first sensor has finished detection
  attachInterrupt(digitalPinToInterrupt(echoPin1), sonarISR, FALLING);
  
    pinMode(PPM_INPUT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT), ppmInterrupt, RISING);
    Serial.begin(SERIAL_BAUD_RATE);

    for(int i = 0; i < NUM_CHANNELS; i++){
        ppmOutput[i] = 1500;
    }
    pinMode(PPM_OUTPUT, OUTPUT);
    Timer1.initialize(1000);
    Timer1.attachInterrupt(&ppmOutputInterrupt, 1000);
}

unsigned long lastPrint = 0;
unsigned int serialInput[NUM_INPUT_CHANNELS] = {0};

void loop() {
    if(ppmInput[MANUAL_CONTROL_CH] > 1200){
        for(int i = 0; i < NUM_CHANNELS; i++){
            ppmOutput[i] = ppmInput[i];
        }
    }else{
        if(Serial.available() >= NUM_INPUT_CHANNELS * 2){
            for(int i = 0; i < NUM_INPUT_CHANNELS; i++){
                serialInput[i] = Serial.read() << 8 | Serial.read();
                //Serial.print(serialInput[i]);
                //Serial.print(", ");
            }
            //Serial.println();
        }
        int i;
        for(i = 0; i < NUM_INPUT_CHANNELS; i++){
            ppmOutput[i] = serialInput[i];
        }
        for(; i < NUM_CHANNELS; i++){
            ppmOutput[i] = 1000;
        }
    }
    
  //Begin code to poll first sonar sensor
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

volatile int currentChannel = 0;
volatile unsigned long lastPulseStart = 0;
volatile unsigned int duration = 0;

void ppmInterrupt(){
    // TODO: Replace with TCNT0 somehow
    unsigned long currentTime = micros();
    duration = currentTime - lastPulseStart;
    lastPulseStart = currentTime;
    if(duration > 3000){
        currentChannel = 0;
    }else{
        ppmInput[currentChannel] = duration;
        currentChannel = (currentChannel + 1) % NUM_CHANNELS;
    }
}

volatile int currentOutputChannel = 0;
volatile unsigned int timeElapsed;

void ppmOutputInterrupt(){
    digitalWrite(PPM_OUTPUT, HIGH);
    digitalWrite(PPM_OUTPUT, LOW);
    currentOutputChannel++;
    if(currentOutputChannel == NUM_CHANNELS){
        Timer1.setPeriod(FRAME_WIDTH - timeElapsed);
        timeElapsed = 0;
        currentOutputChannel = -1;
    }else{
        Timer1.setPeriod(ppmOutput[currentOutputChannel]);
        timeElapsed += ppmOutput[currentOutputChannel];
    }
}
//
//ISR(TIMER1_COMPA_vect){  //leave this alone
//  static boolean state = true;
//  
//  TCNT1 = 0;
//  
//  if (state) {  //start pulse
//    digitalWrite(PPM_OUT, ON_STATE);
//    OCR1A = PULSE_LENGTH * 2;
//    state = false;
//  } else{  //end pulse and calculate when to start the next pulse
//    static byte cur_chan_numb;
//    static unsigned int calc_rest;
//  
//    digitalWrite(PPM_OUT, !ON_STATE);
//    state = true;
//
//    if(cur_chan_numb >= NUM_CHANNELS){
//      cur_chan_numb = 0;
//      calc_rest = calc_rest + PULSE_LENGTH;// 
//      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
//      calc_rest = 0;
//    }
//    else{
//      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
//      calc_rest = calc_rest + ppm[cur_chan_numb];
//      cur_chan_numb++;
//    }     
//  }
//}
//

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

