/***********/
/* PINOUT  */
/*         */
/* SDA 21  */
/* SCL 22  */
/* AD0 GND */
/* 3V3     */
/* INT1 2  */
/* INT2 4  */
/***********/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu1(0x68);  // Dirección por defecto del MPU6050
MPU6050 mpu2(0x69);  // Dirección del segundo MPU6050

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN1 2  // Pin de interrupción para mpu1
#define INTERRUPT_PIN2 4  // Pin de interrupción para mpu2
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady1 = false;  // set true if DMP init was successful
bool dmpReady2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1, q2;           // [w, x, y, z]         quaternion container
VectorFloat gravity1, gravity2;    // [x, y, z]            gravity vector
float ypr1[3], ypr2[3];      // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt1 = false;     // indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt2 = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady1() {
    mpuInterrupt1 = true;
}

void dmpDataReady2() {
    mpuInterrupt2 = true;
}

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

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    mpu2.initialize();
    pinMode(INTERRUPT_PIN1, INPUT);
    pinMode(INTERRUPT_PIN2, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050 #1 connection successful") : F("MPU6050 #1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050 #2 connection successful") : F("MPU6050 #2 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP for mpu1
    Serial.println(F("Initializing DMP for MPU6050 #1..."));
    devStatus1 = mpu1.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(124.00000);
    mpu1.setYGyroOffset(36.00000);
    mpu1.setZGyroOffset(10.00000);
    mpu1.setZAccelOffset(1500.00000); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus1 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu1.CalibrateAccel(6);
        mpu1.CalibrateGyro(6);
        mpu1.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP for MPU6050 #1..."));
        mpu1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN1));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN1), dmpDataReady1, RISING);
        mpuIntStatus1 = mpu1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready for MPU6050 #1! Waiting for first interrupt..."));
        dmpReady1 = true;

        // get expected DMP packet size for later comparison
        packetSize1 = mpu1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed for MPU6050 #1 (code "));
        Serial.print(devStatus1);
        Serial.println(F(")"));
    }

    // load and configure the DMP for mpu2
    Serial.println(F("Initializing DMP for MPU6050 #2..."));
    devStatus2 = mpu2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu2.setXGyroOffset(124.00000);
    mpu2.setYGyroOffset(36.00000);
    mpu2.setZGyroOffset(10.00000);
    mpu2.setZAccelOffset(1500.00000); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus2 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu2.CalibrateAccel(6);
        mpu2.CalibrateGyro(6);
        mpu2.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP for MPU6050 #2..."));
        mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN2));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady2, RISING);
        mpuIntStatus2 = mpu2.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready for MPU6050 #2! Waiting for first interrupt..."));
        dmpReady2 = true;

        // get expected DMP packet size for later comparison
        packetSize2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed for MPU6050 #2 (code "));
        Serial.print(devStatus2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady1 || !dmpReady2) return;

    // read a packet from FIFO for mpu1
    if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer1)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees for mpu1
            mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
            mpu1.dmpGetGravity(&gravity1, &q1);
            mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
            Serial.print("MPU6050 #1 ypr\t");
            Serial.print(ypr1[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr1[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr1[2] * 180/M_PI);
        #endif
    }

    // read a packet from FIFO for mpu2
    if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer2)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees for mpu2
            mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
            mpu2.dmpGetGravity(&gravity2, &q2);
            mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
            Serial.print("MPU6050 #2 ypr\t");
            Serial.print(ypr2[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr2[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr2[2] * 180/M_PI);
            delay(1000);
        #endif
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
