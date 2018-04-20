#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>

#include <MPU9250.h>
#include <Wire.h>
#include <math.h>

//////////////////////////////////////////////////////////////
///// The following values must match those in config.py /////
//////////////////////////////////////////////////////////////

// Number of channels that are sent over serial
#define NUM_INPUT_CHANNELS 4

#define SERIAL_BAUD_RATE 115200

// Total number of sensors on the drone, including remote control inputs
#define NUM_SENSORS 13

// Index of the Leddar sensor
#define LEDDAR_SENSOR_NUM 6

#define COMPASS_SENSOR_NUM 9
// How many sonar sensors
#define NUM_SONAR_SENSORS 2
// Indexes of the sonar sensors
const int SONAR_SENSOR_NUMS[NUM_SONAR_SENSORS] = {7, 8};

#define YAW_SENSOR_NUM 10
#define PITCH_SENSOR_NUM 11
#define ROLL_SENSOR_NUM 12

////////////////
///// PINS /////
////////////////

// Pin for PPM input. Must be either pin 1 or pin 2, because those are the only
// pins that support interrupts.
#define PPM_INPUT 2
// Pin on which to output a PPM signal
#define PPM_OUTPUT 4

//Trigger - used to send signal out from sensor
//Echo - used to recieve signal bounced back from obstacle
const int triggerPin1 = 13;
const int echoPin1 = 3; //interrupt pin

//Values used for second sonar sensor
const int triggerPin2 = 11;
const int echoPin2 = 10;

// Pins of the sonar sensors
const int SONAR_SENSOR_PINS[NUM_SONAR_SENSORS] = {13, 12};

// Channels of remote controller inputs and outputs
// Note: This program uses 0-based indexing, but the remote control, and the LibrePilot
// GCS software use 1-based index. Therefore, channel 0 here corresponds to channel 1 of the
// remote control and GCS, and so on.
#define THROTTLE_CH 2
#define ROLL_CH 0
#define PITCH_CH 1
#define YAW_CH 3
#define AUX_CH 4
// This is the channel for the manual control switch
#define MANUAL_CONTROL_CH 5

///////////////////////////////
///// Misc. Configuration /////
///////////////////////////////

// There are only 6 usable channels, but for some reason, the receiver outputs 8 PPM
// channels, where the last 2 just never change.
#define NUM_CHANNELS 8

// If no serial signal is received in this many milliseconds, then switch to manual control
#define SERIAL_TIMEOUT 500

// Total width of one PPM frame, in microseconds. This was determined by the receiver.
#define FRAME_WIDTH 20000

// Speed of sound in centimeters per microsecond
#define SPEED_OF_SOUND 0.0343

// For sonar sensors
#define MAX_PULSE_DURATION 30000

// Period of IMU updates, in milliseconds
#define IMU_UPDATE_PERIOD 10

//////////////////////////////////
///// Misc. Global Variables /////
//////////////////////////////////

// When the last sonar echo pulse started
volatile unsigned long sonarPulseStart = 0;
// When the last sonar echo pulse ended
volatile unsigned long sonarPulseEnd = 0;

// Index of the current sonar senor that is waiting for a pulse
volatile int currentSonarSensor = 0;

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

// Time that serial information was last received
unsigned long lastSerialReceived = 0;
// True if it's been more than SERIAL_TIMEOUT milliseconds since the last serial input
volatile bool serialDisconnected = true;
volatile bool manualSwitch = false;

float mag_offset[3] = {31.98f, -16.74f, -0.13f};
float mag_softiron_matrix[3][3] = {
    {1.012f, -0.007f, -0.008f},
    {-0.007f, 0.969f, 0.009f},
    {-0.008f, -0.009f, 1.020f}
};

MPU9250 imu(Wire, 0x68);
Mahony filter;

void setup() {
    
    //Added pin mappings for sonar sensors
    pinMode(triggerPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(triggerPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    //Attach sonarISR to interrupt to activate second sonar sensor once
    //first sensor has finished detection
    //attachInterrupt(digitalPinToInterrupt(echoPin1), sonarISR, FALLING);
  
    pinMode(PPM_INPUT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_INPUT), ppmInterrupt, RISING);
    Serial.begin(SERIAL_BAUD_RATE);

    for(int i = 0; i < NUM_CHANNELS; i++){
        ppmOutput[i] = 1500;
    }
    pinMode(PPM_OUTPUT, OUTPUT);

    // Because we're dealing with interrupts, we need to temporarily disable all of them
    noInterrupts();
    // See page 203 of http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
    // for a comprehensive description of what all of this nonsense is.
    // Set Timer2 to CTC (Clear Timer on Compare Match) mode
    TCCR2A = 0b00000010;
    // Set Timer2 divider to 8. This makes each tick 0.5 microseconds. (There's no divider of 16)
    TCCR2B = 0b00000010;
    // This isn't strictly necessary, but it sets the Timer2 counter to 0. (It will immediately resume incrementing,
    // because that's what timers do)
    TCNT2 = 0;
    // Set the output compare to 255, for no apparent reason.
    OCR2A = 0xFF;
    // Enable to Output Compare A interrupt
    TIMSK2 = 0b00000010;
    // Done configuring Timer2, so resume all interrupts.
    interrupts();

    for(int i = 0; i < NUM_SONAR_SENSORS; i++){
        pinMode(SONAR_SENSOR_PINS[i], OUTPUT);
    }

    Wire.begin();
    imu.begin();
    imu.setMagCalX(-20.01, 0.99);
    imu.setMagCalY(20.73, 1.02);
    imu.setMagCalZ(-4.20, 0.99);
    filter.begin(1000 / IMU_UPDATE_PERIOD);
}

volatile unsigned int serialInput[NUM_INPUT_CHANNELS] = {0};

uint16_t sensorBuffer[NUM_SENSORS] = {0};

void loop() {
    if(Serial.available() >= NUM_INPUT_CHANNELS * 2){
        lastSerialReceived = millis();
        serialDisconnected = false;
        for(int i = 0; i < NUM_INPUT_CHANNELS; i++){
            serialInput[i] = Serial.read() << 8 | Serial.read();
            ppmOutput[i] = serialInput[i];
            //Serial.print(serialInput[i]);
            //Serial.print(", ");
        }
        Serial.write(reinterpret_cast<uint8_t*>(sensorBuffer), NUM_SENSORS * 2);
    }else if(millis() - lastSerialReceived > SERIAL_TIMEOUT){
        serialDisconnected = true;
        serialInput[THROTTLE_CH] = 1000;
        serialInput[ROLL_CH] = 1500;
        serialInput[YAW_CH] = 1500;
        serialInput[PITCH_CH] = 1500;
    }

    sensorBuffer[ROLL_CH] = (uint16_t) ppmInput[ROLL_CH];
    sensorBuffer[PITCH_CH] = (uint16_t) ppmInput[PITCH_CH];
    sensorBuffer[YAW_CH] = (uint16_t) ppmInput[YAW_CH];
    sensorBuffer[THROTTLE_CH] = (uint16_t) ppmInput[THROTTLE_CH];
    sensorBuffer[AUX_CH] = (uint16_t) ppmInput[AUX_CH];
    sensorBuffer[MANUAL_CONTROL_CH] = (uint16_t) ppmInput[MANUAL_CONTROL_CH];

    sensorBuffer[LEDDAR_SENSOR_NUM] = millis() % 1000;

    updateIMU();
    updateSonar();

}

const float radToDeg = 180.0f / PI;

void updateIMU(){
    static long lastUpdate = 0;

    if(millis() - lastUpdate > IMU_UPDATE_PERIOD){
        lastUpdate = millis();
        imu.readSensor();
    
        float ax, ay, az, gx, gy, gz, x, y, z, mx, my, mz, heading, yaw, pitch, roll;
        ax = imu.getAccelX_mss();
        ay = imu.getAccelY_mss();
        az = imu.getAccelZ_mss();
        gx = imu.getGyroX_rads() * radToDeg;
        gy = imu.getGyroY_rads() * radToDeg;
        gz = imu.getGyroZ_rads() * radToDeg;
        x = imu.getMagX_uT() - mag_offset[0];
        y = imu.getMagY_uT() - mag_offset[1];
        z = imu.getMagZ_uT() - mag_offset[2];
        mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
        my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
        mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
        
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
        heading = atan2(my, mx) * radToDeg;
        heading = heading < 0 ? heading + 360.0f : heading;
        yaw = filter.getYaw();
        yaw = yaw < 0 ? yaw + 360.0f : yaw;
        pitch = filter.getPitch();
        pitch = pitch < 0 ? pitch + 360.0f : pitch;
        roll = filter.getRoll();
        roll = roll < 0 ? roll + 360.0f : roll;
    
        sensorBuffer[YAW_SENSOR_NUM] = (uint16_t)(yaw * 10);
        sensorBuffer[PITCH_SENSOR_NUM] = (uint16_t)(pitch * 10);
        sensorBuffer[ROLL_SENSOR_NUM] = (uint16_t)(roll * 10);
        sensorBuffer[COMPASS_SENSOR_NUM] = (uint16_t)(heading * 10);
    }
}

void updateSonar(){
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

    //Begin code to poll second sonar sensor 
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
}


// Input on PPM input
void ppmInterrupt(){
    static int currentChannel = 0;
    static unsigned long lastPulseStart = 0;
    static unsigned int duration = 0;

    unsigned long currentTime = micros();
    duration = currentTime - lastPulseStart;
    lastPulseStart = currentTime;
    if(duration > 3000){
        currentChannel = 0;
    }else{
        ppmInput[currentChannel] = duration;
        if(currentChannel == MANUAL_CONTROL_CH){
            manualSwitch = duration > 1200;
        }
        currentChannel = (currentChannel + 1) % NUM_CHANNELS;
    }
}

// Interrupt for PPM output. This gets triggered whenever TCNT2 == OCR2A
// (Timer2 counter == Output compare register 2A)
ISR(TIMER2_COMPA_vect){
    // Total number of microseconds elapsed for this frame
    static uint32_t frameTimeElapsed = 0;
    // Number of clock ticks left for the current pulse
    static uint16_t pulseTicksLeft = 0;
    // Channel number of the current pulse
    static uint8_t currentChannel = 0;

    // Here's how this works: The timer is only 8 bits, which is only 128 microseconds (us).
    // But PPM pulses are up to 2000us apart. One option would be to increase the timer prescaler,
    // so each tick is multiple microseconds, but this would decrease resolution. So, I just keep
    // track of how many ticks have elapsed for the current pulse, and just keep setting the
    // output compare register (OCR) to 255, until I have less than 255 ticks left.

    if(!pulseTicksLeft){ // This means that the current pulse is finished.
        currentChannel++;
        // Entering the "Dead space" between frames
        if(currentChannel >= NUM_CHANNELS){
            currentChannel = -1;
            pulseTicksLeft = (FRAME_WIDTH - frameTimeElapsed);
            frameTimeElapsed = 0;
        }else if (serialDisconnected || manualSwitch){ // Something is wrong, so enter manual control mode
            pulseTicksLeft = ppmInput[currentChannel];
            frameTimeElapsed += pulseTicksLeft;
        }else{
            pulseTicksLeft = ppmOutput[currentChannel];
            frameTimeElapsed += pulseTicksLeft;
        }
        // Double the number of ticks, because one tick = 0.5 microseconds
        pulseTicksLeft <<= 1;
        // One quick pulse
        digitalWrite(PPM_OUTPUT, HIGH);
        digitalWrite(PPM_OUTPUT, LOW);
    }

    if(pulseTicksLeft <= 255){
        OCR2A = pulseTicksLeft;
        pulseTicksLeft = 0;
    }else{
        OCR2A = 255;
        pulseTicksLeft -= 255;
    }
}

