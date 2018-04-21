// Libraries needed:
// https://github.com/PaulStoffregen/MahonyAHRS
// https://github.com/PaulStoffregen/MadgwickAHRS
// https://github.com/bolderflight/MPU9250
// For each one:
//  1. On github, click "Clone or Download" > "Download Zip"
//  2. In Arduino, click Sketch > Include Library > Add .Zip Library
// To run this code, upload it to the arduino, then click Tools > Serial Monitor
//    (Make sure to set the serial monitor baud rate in the lower-right corner to 115200)
//    (Also, if you open the serial monitor while the code is still uploading, it messes
//     up. So just wait for it to finish uploading before opening the serial monitor)
//    When the arduino first starts up, it spends a few seconds calibrating the gyroscope.
//    Make sure the motion sensor is perfectly still and flat on a table while this is happening.
// There some single-letter abbreviations scattered throughout:
//  M: Magnetometer
//  G: Gyroscope
//  A: Accelerometer
//
// See the loop() function for a list of different things I've tried.

#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>

#include <MPU9250.h>
#include <Wire.h>
#include <math.h>


MPU9250 imu(Wire, 0x68);
Mahony filter;

int stat;

#define BUFFER_SIZE 20

#define G_PER_COUNT            0.0001220703125f  // = 1/8192
#define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
#define UT_PER_COUNT           0.1f

void setup() {
    Wire.begin();
    Serial.begin(115200);
    imu.begin();
    imu.setMagCalX(-20.01, 0.99);
    imu.setMagCalY(20.73, 1.02);
    imu.setMagCalZ(-4.20, 0.99);
    // Uncomment below to calibrate the magnetometer.
    // While calibrating, I think you're supposed to slowly wave it around
    // in all directions, so it can get measurements of all possible orientations.
    // Who knows, I may be wrong...
//    Serial.println("Calibrating mag...");
//    imu.calibrateMag();
//    Serial.println("Done!");
//    Serial.print("X: (");
//    Serial.print(imu.getMagBiasX_uT());
//    Serial.print(", ");
//    Serial.print(imu.getMagScaleFactorX());
//    Serial.print("), Y: (");
//    Serial.print(imu.getMagBiasY_uT());
//    Serial.print(", ");
//    Serial.print(imu.getMagScaleFactorY());
//    Serial.print("), Z: (");
//    Serial.print(imu.getMagBiasZ_uT());
//    Serial.print(", ");
//    Serial.print(imu.getMagScaleFactorZ());
//    Serial.println(")");
//    delay(20000);
    // Uncomment below to calibrate the accelerometer.
    // I have no idea whatsoever how this is supposed to work.
//    Serial.println("Calibrating accel...");
//    imu.calibrateAccel();
//    Serial.print("Bias: (");
//    Serial.print(imu.getAccelBiasX_mss());
//    Serial.print(", ");
//    Serial.print(imu.getAccelBiasY_mss());
//    Serial.print(", ");
//    Serial.print(imu.getAccelBiasZ_mss());
//    Serial.println(")");
//    delay(10000);
    filter.begin(10);
}

float radToDeg = 180.0f / PI;
int loopCount = 0;
long loopStart = -1;

// Some values taken from the MotionCal program on my laptop.
// These are only used in the sensorFusion() function.
// I have temporarily set them to do nothing.

float mag_offset[3] = {31.98f, -16.74f, -0.13f};
float mag_softiron_matrix[3][3] = {
    {1.012f, -0.007f, -0.008f},
    {-0.007f, 0.969f, 0.009f},
    {-0.008f, -0.009f, 1.020f}
};
//float mag_offset[3] = {0, 0, 0};
// To make this matrix have no effect, set it to
// this identity matrix.
//float mag_softiron_matrix[3][3] = {
//    {1, 0, 0},
//    {0, 1, 0},
//    {0, 0, 1}
//};


void loop() {
    // Uncomment exactly one of these functions to try some stuff out.
    
    // This is just an attempt to find the minumum and maximum readings of the
    // magnetometer while waving it around in all directions, so I can try to
    // manually offset them to be centered around 0.
    //getMinAndMax();

    // This uses the Madgwick or Mahony filter to try to fuse all three sensors
    // together. It's a bit screwy...
    // Note: When you first start it, it slowly flips over to roll = -180 degrees.
    // This is because the filter for some reason expects the Z axis of the sensor
    // to be pointed downward, but in its current orientation, it points upward.
    sensorFusion();

    // Plots all three axes of the magnetometer. To do this, open the serial plotter
    // instead of the serial monitor. (Tools > Serial Plotter)
    //plot();

    // If you want to use the MotionCal program on your laptop, you'll have to use
    // this function, because MotionCal expects the data in a specific format.
    //calibrate();

    //delay(100);
}

void plot(){
    imu.readSensor();
    Serial.print(imu.getMagX_uT());
    Serial.print(",");
    Serial.print(imu.getMagY_uT());
    Serial.print(",");
    Serial.print(imu.getMagZ_uT());
    Serial.println(",");
    delay(100);
}

float mins[3] = {1000, 1000, 1000};
float maxs[3] = {-1000, -1000, -1000};
float tots[3] = {0, 0, 0};

float totalMag = 0.0f;
int numLoops = 0;

void getMinAndMax(){
    imu.readSensor();
    numLoops++;
    float data[3];
    data[0] = imu.getMagX_uT() - mag_offset[0];
    data[1] = imu.getMagY_uT() - mag_offset[1];
    data[2] = imu.getMagZ_uT() - mag_offset[2];
    float mag = 0;
    for(int i = 0; i < 3; i++){
        mag += (data[i] * data[i]);
        tots[i] += data[i];
    }
    mag = sqrt(mag);
    totalMag += mag;
//    Serial.print("Magnitude: ");
//    Serial.print(mag);
//    Serial.print(" Avg mag: ");
//    Serial.print(totalMag / numLoops);

    Serial.print("Data: (");
    Serial.print(data[0]);
    Serial.print(", ");
    Serial.print(data[1]);
    Serial.print(", ");
    Serial.print(data[2]);
    Serial.print(") ");

    Serial.print("Avg: (");
    Serial.print(tots[0] / numLoops);
    Serial.print(", ");
    Serial.print(tots[1] / numLoops);
    Serial.print(", ");
    Serial.print(tots[2] / numLoops);
    Serial.print(") ");

    
    for(int i = 0; i < 3; i++){
        if(data[i] < mins[i]){
            mins[i] = data[i];
        }
        if(data[i] > maxs[i]){
            maxs[i] = data[i];
        }
    }

    Serial.print(" Max: (");
    Serial.print(maxs[0]);
    Serial.print(", ");
    Serial.print(maxs[1]);
    Serial.print(", ");
    Serial.print(maxs[2]);
    Serial.print("), Min: (");
    Serial.print(mins[0]);
    Serial.print(", ");
    Serial.print(mins[1]);
    Serial.print(", ");
    Serial.print(mins[2]);
    Serial.println(")");

    delay(100);
}

void sensorFusion(){
    imu.readSensor();

    float ax, ay, az, gx, gy, gz, x, y, z, mx, my, mz;
    ax = imu.getAccelX_mss();
    ay = imu.getAccelY_mss();
    az = imu.getAccelZ_mss();
    gx = imu.getGyroX_rads() * radToDeg;
    gy = imu.getGyroY_rads() * radToDeg;
    gz = imu.getGyroZ_rads() * radToDeg;
    mx = imu.getMagX_uT();
    my = imu.getMagY_uT();
    mz = imu.getMagZ_uT();
    x = imu.getMagX_uT() - mag_offset[0];
    y = imu.getMagY_uT() - mag_offset[1];
    z = imu.getMagZ_uT() - mag_offset[2];
    mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
    
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    //filter.updateIMU(gx, gy, gz, ax, ay, az);
    float yaw = filter.getYaw();
    float pitch = -filter.getPitch();
    float roll = filter.getRoll();
    roll = roll > 0 ? 180.0f - roll : -180.0f - roll;
    roll = -roll;
    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Roll: ");
    Serial.print(roll);

//    Serial.print(", M: (");
//    Serial.print(mx);
//    Serial.print(", ");
//    Serial.print(my);
//    Serial.print(", ");
//    Serial.print(mz);
//
//    Serial.print("), A: (");
//    Serial.print(ax);
//    Serial.print(", ");
//    Serial.print(ay);
//    Serial.print(", ");
//    Serial.print(az);

    
    
    float sinphi = sin(roll / radToDeg);
    float sintheta = sin(pitch / radToDeg);
    float cosphi = cos(roll / radToDeg);
    float costheta = cos(pitch / radToDeg);

    Serial.print(", Trig: (");
    Serial.print(sinphi);
    Serial.print(", ");
    Serial.print(cosphi);
    Serial.print(", ");
    Serial.print(sintheta);
    Serial.print(", ");
    Serial.print(costheta);
    
    // Magnetometer tilt compensation
    // (Equations from https://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf)
    float tmx, tmy, tmz;
    tmx = (mx * costheta) + (my * sintheta * sinphi) + (mz * sintheta * cosphi);
    tmy = (my * cosphi) - (mz * sinphi);
    //tmz = (-mx * sintheta) + (my * costheta * sinphi) + (mz * costheta * cosphi);

    float heading = atan2(my, mx) * radToDeg;
    if(heading < 0){
        heading += 360.0f;
    }
    float betterheading = atan2(tmy, tmx) * radToDeg;
    if(betterheading < 0){
        betterheading += 360.0f;
    }
    //heading = updateAverageHeading(heading);
    Serial.print("), Raw Heading: ");
    Serial.print(heading);
    Serial.print(", Tilt-Corrected Heading: ");
    Serial.print(betterheading);
    Serial.println("");

    delay(100);
}

void calibrate(){
    imu.readSensor();

    float ax, ay, az, gx, gy, gz, x, y, z, mx, my, mz;
    ax = imu.getAccelX_mss();
    ay = imu.getAccelY_mss();
    az = imu.getAccelZ_mss();
    gx = imu.getGyroX_rads();// * radToDeg;
    gy = imu.getGyroY_rads();// * radToDeg;
    gz = imu.getGyroZ_rads();// * radToDeg;
    mx = imu.getMagX_uT();
    my = imu.getMagY_uT();
    mz = imu.getMagZ_uT();
    
    Serial.print("Raw:");
    Serial.print((int) (ax * 8192));
    Serial.print(',');
    Serial.print((int) (ay * 8192));
    Serial.print(',');
    Serial.print((int) (az * 8192));
    Serial.print(',');
    Serial.print((int) (gx * 16));
    Serial.print(',');
    Serial.print((int) (gy * 16));
    Serial.print(',');
    Serial.print((int) (gz * 16));
    Serial.print(',');
    Serial.print((int) (mx * 10));
    Serial.print(',');
    Serial.print((int) (my * 10));
    Serial.print(',');
    Serial.print((int) (mz * 10));
    Serial.println();  
}

// Just performs a rolling average of whatever values you give it.
// Might be useful, might not.
double updateAverageHeading(double heading){
    static double headingBuffer[BUFFER_SIZE] = {0.0};
    static int currentBufferLocation = 0;

    headingBuffer[currentBufferLocation] = heading;
    currentBufferLocation = (currentBufferLocation + 1) % BUFFER_SIZE;

    double avg = 0;
    for(int i = 0; i < BUFFER_SIZE; i++){
        avg += headingBuffer[i];
    }
    return avg / (double) BUFFER_SIZE;
}

