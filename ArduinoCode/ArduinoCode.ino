#include <TimerOne.h>

// There are only 6 usable channels, but for some reason, the receiver outputs 8 PPM
// channels, where the last 2 just never change.
#define NUM_CHANNELS 8
#define PPM_INPUT 2

// Total width of one PPM frame, in microseconds
#define FRAME_WIDTH 20000
// Pin on which to output a PPM signal
#define PPM_OUTPUT 8

// Channels of remote controller inputs and outputs
// Note: This program uses 0-based indexing, but the remote control, and the LibrePilot
// GCS software use 1-based index. Therefore, channel 0 here corresponds to channel 1 of the
// remote control and GCS, and so on.
#define THROTTLE_CH 2
#define ROLL_CH 0
#define PITCH_CH 1
#define YAW_CH 3
#define ARM_CH 4
#define FLIGHT_MODE_CH 5

// Constantly updates to store the current values of the ppm inputs
volatile unsigned int ppmInput[NUM_CHANNELS] = {0};
// Change this array at any time to change the PPM output
volatile unsigned int ppmOutput[NUM_CHANNELS] = {0};

void setup() {
    pinMode(PPM_INPUT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT), ppmInterrupt, RISING);
    Serial.begin(115200);

    for(int i = 0; i < NUM_CHANNELS; i++){
        ppmOutput[i] = 1500;
    }
    pinMode(PPM_OUTPUT, OUTPUT);
    Timer1.initialize(1000);
    Timer1.attachInterrupt(&ppmOutputInterrupt, 1000);
}

unsigned long lastPrint = 0;

void loop() {

//    ppmOutput[3] = ((millis() % 2000) / 2) + 1000;
//
//    if(millis() - lastPrint > 500){
        lastPrint = millis();
        for(int i = 0; i < NUM_CHANNELS; i++){
            Serial.print(ppmInput[i]);
            //sum += ppmInput[i];
            Serial.print(", ");
        }
        //Serial.print(sum);
        Serial.println("");
        delay(500);
//    }
//    delay(500);
//    for(int i = 0; i < NUM_CHANNELS; i++){
//        ppmOutput[i] = ppmInput[i];
//    }
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

