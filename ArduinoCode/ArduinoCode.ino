
#define NUM_CHANNELS 6
#define PPM_PIN 8

volatile unsigned long inputPulseWidths[NUM_CHANNELS] = {0};

/*
 * PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 * 
 * With modifications by Timothy Scott
 */

//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define ON_STATE 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define PPM_OUT 10  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[NUM_CHANNELS];

void setup() {
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, FALLING);
    Serial.begin(115200);
    pinMode(8, OUTPUT);

    // initiallize default ppm values
    for(int i=0; i < NUM_CHANNELS; i++){
        ppm[i] = CHANNEL_DEFAULT_VALUE;
    }
    
    pinMode(PPM_OUT, OUTPUT);
    digitalWrite(PPM_OUT, !ON_STATE);  //set the PPM signal pin to the default state (off)
    
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    
    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();

}



void loop() {

    if((millis() / 3000) % 2){
        for(int i = 0; i < NUM_CHANNELS; i++){
            ppm[i] = 1000 + (i * 200);
        }
    }else{
        for(int i = 0; i < NUM_CHANNELS; i++){
            ppm[i] = 1000;
        }
    }

    for(int i = 0; i < NUM_CHANNELS; i++){
        Serial.print(inputPulseWidths[i]);
        Serial.print(", ");
    }
    Serial.println("");
    delay(1000);
}

volatile int currentChannel = 0;
volatile unsigned long lastPulseStart = 0;

void ppmInterrupt(){
    // TODO: Replace with TCNT1
    unsigned long currentTime = micros();
    inputPulseWidths[currentChannel] = currentTime - lastPulseStart;
    lastPulseStart = currentTime;
    currentChannel = (currentChannel + 1) % NUM_CHANNELS;
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(PPM_OUT, ON_STATE);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PPM_OUT, !ON_STATE);
    state = true;

    if(cur_chan_numb >= NUM_CHANNELS){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}


