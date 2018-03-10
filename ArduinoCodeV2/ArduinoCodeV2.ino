#include <SoftwareSerial.h>
#include "LibrePilotSerial.h"
#include "manualcontrolcommand.h"
#include "flightstatus.h"
#include "gcsreceiver.h"

#define FLIGHT_SERIAL_RX 8
#define FLIGHT_SERIAL_TX 9

SoftwareSerial flightSerial(FLIGHT_SERIAL_RX, FLIGHT_SERIAL_TX);
LibrePilotSerial lps(&flightSerial);

void setup() {
  Serial.begin(57600);
  lps.serial->begin(57600);

//  ManualControlCommandDataUnion.data.Throttle = 0;
//  ManualControlCommandDataUnion.data.Pitch = 0;
//  ManualControlCommandDataUnion.data.Yaw = 0;
//  ManualControlCommandDataUnion.data.Roll = 0;

    GCSReceiverDataUnion.data.Channel[0] = 1000;
    GCSReceiverDataUnion.data.Channel[1] = 1500;
    GCSReceiverDataUnion.data.Channel[2] = 1500;
    GCSReceiverDataUnion.data.Channel[3] = 1500;
    GCSReceiverDataUnion.data.Channel[4] = 1500;
    GCSReceiverDataUnion.data.Channel[5] = 2000;
}

long lastUpdate = 0;

void loop() {
    
    uint16_t throttle;

    if(millis() > 1000){
        throttle = 1000 + (millis() % 1000);
    }else{
        throttle = 1000;
    }

    //Serial.println(throttle);
    //lps.send(MANUALCONTROLCOMMAND_OBJID, ManualControlCommandDataUnion.arr, MANUALCONTROLCOMMAND_NUMBYTES);
    GCSReceiverDataUnion.data.Channel[0] = throttle;
    lps.send(GCSRECEIVER_OBJID, GCSReceiverDataUnion.arr, GCSRECEIVER_NUMBYTES);

    lps.request(FLIGHTSTATUS_OBJID);
    boolean ok = lps.receive(FLIGHTSTATUS_OBJID, FlightStatusDataUnion.arr, 200);

    if(millis() - lastUpdate > 1000){
        Serial.print(millis());
        Serial.print("  ");
        if(ok){
            Serial.print("Result fs: ");
            Serial.print(FlightStatusDataUnion.data.Armed);
            if(FlightStatusDataUnion.data.Armed == FLIGHTSTATUS_ARMED_ARMED){
                Serial.println(" (Armed)");  
            }else{
                Serial.println(" (Not Armed)");
            }
        }else{
            Serial.println("Not OK!");
        }
        lastUpdate = millis();
    }
//    
//    if(ok) {
//        if(FlightStatusDataUnion.data.Armed == FLIGHTSTATUS_ARMED_DISARMED) {
//            analogWrite(ledPinr, 0);
//            analogWrite(ledPiny, 0);
//            analogWrite(ledPing, 255);
//        } else if (FlightStatusDataUnion.data.Armed == FLIGHTSTATUS_ARMED_ARMING) {
//            analogWrite(ledPinr, 0);
//            analogWrite(ledPiny, 255);
//            analogWrite(ledPing, 0);
//        } else if (FlightStatusDataUnion.data.Armed == FLIGHTSTATUS_ARMED_ARMED) {
//            analogWrite(ledPinr, 255);
//            analogWrite(ledPiny, 0);
//            analogWrite(ledPing, 0);
//        }
//    }
    
    delay(50);
}

