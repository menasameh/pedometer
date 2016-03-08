
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void displayOnLcd();
void getAccData();
void runAlgo();

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

#define BUZZER_PIN 11
#define READINGS_COUNT 200

int readingsNum = 0, addr = 0, th=200;
////////////////////////////////////variables//////////////////////////////////
int sample_cnt = 0;
int sample_new[3] , sample_old[3] , max[3] , min[3];
int steps = 0;
int active_axis=0;
int threshold;
int precision=555;
int timer=0;
unsigned long timerStart=0;
float distance=0;
int sec_2=0;
int steps_2=0;
float height=1.87;
float speed_2 , speed_sum=0 , speed_avg=0;
float stride;
float cal_2=0  , cal_sum=0;
int weight = 1800;
int cnt_2=0;

int pos[3];
int avg[3];
int cnt=0;




volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(9600);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  for(int i=0;i<3;i++){
    avg[i]=0 ; max[i]=-1e9 ; min[i]=1e9;
  }
// set up the LCD's number of columns and rows:
lcd.begin(16, 2);
// Print a message to the LCD.
lcd.setCursor(0,0);
lcd.print("Loading...");
    mpu.initialize();
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(5, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
   
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
}

void loop() {

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        if (timer>200)
          displayOnLcd();
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
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
       if(timer < 200){
        
        getAccData();
        
       }
       else if(timer == 200){
          lcd.setCursor(0,0);
          lcd.print("Steps :   ");
//          unsigned long r = millis(); 
//        lcd.setCursor(0,0);
//        lcd.print("       "); 
          runAlgo();
//          lcd.setCursor(0,0);
//        lcd.print(millis()-r);
//        while(1);
       }
       else{
         runAlgo();
       }
       timer++;
       sec_2++;
       if(sec_2*5 == 2000){
        sec_2=0;
        if(steps_2 >= 0 && steps_2 <= 2){
          stride = height/5;
          
        }
        else if(steps_2 >= 2 && steps_2 <= 3){
           stride = height/4;
        }
         else if(steps_2 >= 3 && steps_2 <= 4){
          stride = height/3;
        }
         else if(steps_2 >= 4 && steps_2 <= 5){
          stride = height/2;
        }
         else if(steps_2 >= 5 && steps_2 <= 6){
          stride = height/1.2;
        }
         else if(steps_2 >= 6 && steps_2 <= 8){
          stride = height;
        }
        else{
          stride = height*1.2;
        }
        distance+=stride*steps_2;
        speed_2 = steps_2 * stride / 2;
        if(steps_2){
          cal_2 = speed_2 * weight / 400;
        }
        else{
          cal_2 = weight / 1800;
        }
        cal_sum += cal_2;
        speed_sum += speed_2;
        speed_avg = speed_sum / cnt_2;
        // lcd.setCursor(0,1);
        // lcd.print(distance);
        steps_2=0;
        cnt_2++;
       }
    }
}



void runAlgo(){
  //read acceleration data into 'pos' array
  getAccData();
  
    cnt++;
    for(int i=0;i<3;i++)
      avg[i]+=pos[i];
    if(cnt == 4){
      cnt=0;
      sample_cnt++;
      // for(int i=0;i<3;i++){ // edited 
      //   avg[i]/=4;
      // }
      if(sample_cnt == 1){
        for(int i=0;i<3;i++)
          sample_new[i] = sample_old[i] = avg[i];
      }
      else{
        for(int i=0;i<3;i++){
          sample_old[i] = sample_new[i];
        }
      }
      for(int i=0;i<3;i++){
        max[i] = max(max[i] , avg[i]);
        min[i] = min(min[i] , avg[i]);
      }
      if(sample_cnt==50){
        sample_cnt=0;
        if(max[0]-min[0] > max[1]-min[1] && max[0]-min[0] > max[2]-min[2])
          active_axis=0;
        else if(max[1]-min[1] > max[2]-min[2] && max[1]-min[1] > max[0]-min[0])
          active_axis=2;
        else 
          active_axis=3;
        threshold = (max[active_axis] + min[active_axis])/2;
        for(int i=0;i<3;i++){
            max[i]=-1e9 ; min[i]=1e9;
        }
      }
      if(abs(sample_new[0] - avg[0]) + abs(sample_new[1] - avg[1]) + abs(sample_new[2] - avg[2]) > precision){
         for(int i=0;i<3;i++)
           sample_new[i] = avg[i];
      }  
      
      //Serial.print("%d , %d , %d         ,%d" ,avg[0] , avg[1] , avg[2] ,    threshold);   
      Serial.print(avg[0]);
      Serial.print(",");
      Serial.print(avg[1]);
      Serial.print(",");
      Serial.print(avg[2]);
      Serial.print(",");
      Serial.print(threshold);
      Serial.print(",");
      Serial.println(steps);
         
      if(sample_new[active_axis] < threshold && threshold < sample_old[active_axis] && threshold > 800){ // edited 
         if(millis() - timerStart > 12000){ // edited from 200
           steps++;
           steps_2++;
           timerStart = millis();
           
         }
      }
      for(int i=0;i<3;i++)
        avg[i] = 0;
    }
}


void getAccData(){
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
  pos[0] = aaWorld.x;
  pos[1] = aaWorld.y;
  pos[2] = aaWorld.z;
}

void beep(){
  digitalWrite(11, HIGH);
  delay(100);
  digitalWrite(11, LOW);
}

void displayOnLcd(){
        /////////////////////////////////////display on screen /////////////////////////////////////
  lcd.setCursor(7,0);
  lcd.print(steps);
}

