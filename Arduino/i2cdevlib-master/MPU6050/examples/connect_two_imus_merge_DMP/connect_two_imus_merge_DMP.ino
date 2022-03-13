/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1, mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus1, devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1, packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1, fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64], fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1, q2;           // [w, x, y, z]         quaternion container
VectorInt16 aa1, aa2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal1, aaReal2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld1, aaWorld2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity1, gravity2;    // [x, y, z]            gravity vector
float euler1[3], euler2[3];         // [psi, theta, phi]    Euler angle container
float ypr1[3], ypr2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket1[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
uint8_t teapotPacket2[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);
    mpu2.initialize();
    pinMode(INTERRUPT_PIN, INPUT); 

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus1 = mpu1.dmpInitialize();
    devStatus2 = mpu2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(34);
    mpu1.setYGyroOffset(-35);
    mpu1.setZGyroOffset(-23);
    mpu1.setZAccelOffset(1632); // 1688 factory default for my test chip

    mpu2.setXGyroOffset(34);
    mpu2.setYGyroOffset(-35);
    mpu2.setZGyroOffset(-23);
    mpu2.setZAccelOffset(1632);

    // make sure it worked (returns 0 if so)
    if ( (devStatus1 == 0) & (devStatus2 ==0) ) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu1.CalibrateAccel(6);
        mpu1.CalibrateGyro(6);
        mpu1.PrintActiveOffsets();

	mpu2.CalibrateAccel(6);
        mpu2.CalibrateGyro(6);
        mpu2.PrintActiveOffsets();

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);
	mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus1 = mpu1.getIntStatus();
	mpuIntStatus2 = mpu2.getIntStatus();


        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize1 = mpu1.dmpGetFIFOPacketSize();
	packetSize2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus1, devStatus2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer1)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
            Serial.print("quat 1\t");
            Serial.print(q1.w);
            Serial.print("\t");
            Serial.print(q1.x);
            Serial.print("\t");
            Serial.print(q1.y);
            Serial.print("\t");
            Serial.println(q1.z);
		
        #endif


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
            mpu1.dmpGetGravity(&gravity1, &q1);
            mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
            Serial.print("ypr sensor 1: \t");
            Serial.print(ypr1[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr1[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr1[2] * 180/M_PI);

	    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
            mpu2.dmpGetGravity(&gravity2, &q2);
            mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
            Serial.print("ypr sensor 2: \t");
            Serial.print(ypr2[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr2[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr2[2] * 180/M_PI);
		
        #endif


    


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
