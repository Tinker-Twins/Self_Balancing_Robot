// ================================================================
// ===                        LIBRARIES                         ===
// ================================================================


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>


// ================================================================
// ===                           DMP                            ===
// ================================================================


MPU6050 mpu;  // Default I2C address is 0x68

#define OUTPUT_READABLE_YAWPITCHROLL  // to see the yaw, pitch, roll angles (in degrees) calculated from the quaternions coming from the FIFO

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                           PID                            ===
// ================================================================


double Setpoint;
double Input;
double Output;
double Kp=10.5, Ki=70, Kd=0.35;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// ================================================================
// ===                          MOTORS                          ===
// ================================================================


#define En1 6  //Enable (PWM) pin for motor 1
#define En2 11  //Enable (PWM) pin for motor 2

#define M11 7 //Control pin 1 for motor 1
#define M12 8 //Control pin 2 for motor 1
#define M21 9 //Control pin 1 for motor 2
#define M22 10 //Control pin 2 for motor 2

int pwmM1;  //Two different PWM values take care of the offset the two motors have due to manufacturing defect
int pwmM2;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Set gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //PID
    Setpoint=75.0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetSampleTime(12.5);

    //Motors
    pinMode(En1,OUTPUT);
    pinMode(En2,OUTPUT);
    pinMode(M11,OUTPUT);
    pinMode(M12,OUTPUT);
    pinMode(M21,OUTPUT);
    pinMode(M22,OUTPUT);

    delay(10000); //delay is added in order to stabilize accelerometer readings
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            double Pitch=ypr[1]*180/M_PI;
            Serial.print("Pitch:\t");
            Serial.println(Pitch);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        //PID
        Input=Pitch;
        myPID.Compute();
        //Serial.print(Input);
        //Serial.print(" ");
        //Serial.println(Output);

        //Motor Control
        if (Output>10 && Output<=255){
          pwmM1=map(Output,0,255,100,255);
          pwmM2=map(Output,0,255,50,255);

          analogWrite(En1,pwmM1);
          analogWrite(En2,pwmM2);

          digitalWrite(M11,HIGH);
          digitalWrite(M12,LOW);
          digitalWrite(M21,HIGH);
          digitalWrite(M22,LOW);
        }

        else if (Output>=-255 && Output<-10){
          pwmM1=map(abs(Output),0,255,100,255);
          pwmM2=map(abs(Output),0,255,50,255);

          analogWrite(En1,pwmM1);
          analogWrite(En2,pwmM2);

          digitalWrite(M11,LOW);
          digitalWrite(M12,HIGH);
          digitalWrite(M21,LOW);
          digitalWrite(M22,HIGH);
        }

        else{
          digitalWrite(M11,LOW);
          digitalWrite(M12,LOW);
          digitalWrite(M21,LOW);
          digitalWrite(M22,LOW);
        }

        //Serial.println(pwmM1);
        //Serial.println(pwmM2);
    }
}
