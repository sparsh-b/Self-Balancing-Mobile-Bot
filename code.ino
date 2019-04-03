// ================================================================
// ===               Library to read from MPU-6050              ===
// ================================================================

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



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

  float e=0;
  float epr=0;
  float eps=0;
  float i=0;
  //kp, ki, kd values are calibrated for the particular body construction used.
  float kd=5.95;
  float ki=0.3;
  float kp=6.2;
  int flag=1;
  unsigned long tcount=0;
  byte A,B,C,D;



void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Offsets are calibrated for the particular Body Construction used.
    mpu.setXGyroOffset(6);
    mpu.setYGyroOffset(10);
    mpu.setZGyroOffset(11);
    mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set DMP Ready flag so the main loop() function knows it's okay to use it
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, do nothing
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow 
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
        #endif

//The gyro values given by MPU-6050 take around 30 seconds of time to become stable.        
  if(millis()>30000)
  {
    if((ypr[1]* 180/M_PI)>3.45 && (ypr[1]* 180/M_PI)<4.95) {bt_input();} //implement direction control when the bot is close to upright position (0.55 degrees about central position of 4.2 degrees, i.e, between 3.45 & 4.95 degrees).
    else //else implement PID control to balance the bot.
    {
      e=e+((4.2)-(ypr[1] * 180/M_PI)); //error variable to be used in PID control
     if((millis()-tcount>30) || flag == 1)
  {
    epr=(4.2)-(ypr[1] * 180/M_PI);//error in present state
    i=(kp*epr)+kd*(epr-eps)+ki*(e);
    Serial.print(i);
    eps=epr;//error in past state
    tcount=millis();
    flag=0;
    if((i<127 && i>-127) || i==127 || i==-127)
    {
    analogWrite(5,127+(int)i);
    analogWrite(6,127-(int)i);
    analogWrite(10,127+(int)i);
    analogWrite(11,127-(int)i);
    }
    else if(i>127)
    {
    analogWrite(5,255);
    analogWrite(6,0);
    analogWrite(10,255);
    analogWrite(11,0);
    }
    else if(i<-127)
    {
    analogWrite(5,0);
    analogWrite(6,255);
    analogWrite(10,0);
    analogWrite(11,255);
    }
  }
  Serial.print("\n");
    }
  }

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
//=====================================================================end of PID Control=================================================================

//=============================================================Function to process information received by HC-05=====================================
void bt_input(){
   if(Serial.available()>=4){

    // if the first byte read is not the first byte sent from the bluetooth skip it and read the next byte
    while(1){
      A = Serial.read();  //Direction
      // check if the first byte value corresponds to those sent by the app over bluetooth
      if(A==241 || A==242 || A==243)
        break;
    }  
    // read the next 3 bytes as well
    B = Serial.read();  //Speed
    C = Serial.read();  //Angle
    D = Serial.read();  //Special buttons

    // send the read commands to the monitor
    Serial.print(A);
    Serial.print(" ");
    Serial.print(B);
    Serial.print(" ");
    Serial.print(C);
    Serial.print(" ");
    Serial.print(D);
    Serial.println(" ");  

    // use the input data to move the robot
    move_robot();
   }
}


void move_robot(){

  // if both joysticks are at center
  if(A==243 && C==89)
    stop_robot();

  // forward
  else if(A==241 && C==89)
    forward(B);
    
  // reverse
  else if(A==242 && C==89)
    back(B);

  //left
  else if(C<89)
    turn_left();

  //right
  else if(C>89)
    turn_right();
  
  else
    stop_robot();
}

void stop_robot(){
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
}

void forward(int wheel_speed){
  analogWrite(5,wheel_speed);
  digitalWrite(6,LOW);
  analogWrite(10,wheel_speed);
  digitalWrite(11,LOW);
}

void back(int wheel_speed){
  analogWrite(5,(255-wheel_speed));
  digitalWrite(6,HIGH);
  analogWrite(10,(255-wheel_speed));
  digitalWrite(11,HIGH);
}

void turn_right(){
    digitalWrite(5,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(10,LOW);
    digitalWrite(11,HIGH);
}

void turn_left(){
    digitalWrite(5,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(10,HIGH);
    digitalWrite(11,LOW);
}
