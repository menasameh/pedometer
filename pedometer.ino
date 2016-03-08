
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

void print_to_lCD();
void getAccData();
void process_input(int i);
int read_LCD_buttons();
void runALgo();


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

#define MOVING_SIZE 20
#define SAMPLE_SIZE 50
#define MAX_MIN_DIFF 2000
#define NOW_PREV_DIFF 100

int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define settings 0

int st[3] , pos[3];
int samples[3][SAMPLE_SIZE];
int moving[3][MOVING_SIZE];
int idx_sample[3];
int max_min_diff[3];

int readingsNum = 0, addr = 0, th=200;
////////////////////////////////////variables//////////////////////////////////

int state =0,Spage=0,Mpage=0,button_pressed=0;

int timer=0;
unsigned long timerStart=0;
int sec_2=0;
int steps_2=0;
float speed_2 , speed_sum=0 , speed_avg=0;
float stride;
float cal_2=0  , cal_sum=0;
int cnt_2=0;



int steps;
float height = 1.60;
float weight = 80;
float distance=0;
float cal=0;



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

        
        
    }
    if (timer>200){
          process_input(read_LCD_buttons());
          print_to_lCD();
        }
        else if(timer == 200){
          lcd.clear();
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
       else if(timer >= 200){
          if(state){
            runALgo();
          }
       }
       timer++;
//       sec_2++;
//       if(sec_2*5 == 2000){
//        sec_2=0;
//        if(steps_2 >= 0 && steps_2 <= 2){
//          stride = height/5;
//          
//        }
//        else if(steps_2 >= 2 && steps_2 <= 3){
//           stride = height/4;
//        }
//         else if(steps_2 >= 3 && steps_2 <= 4){
//          stride = height/3;
//        }
//         else if(steps_2 >= 4 && steps_2 <= 5){
//          stride = height/2;
//        }
//         else if(steps_2 >= 5 && steps_2 <= 6){
//          stride = height/1.2;
//        }
//         else if(steps_2 >= 6 && steps_2 <= 8){
//          stride = height;
//        }
//        else{
//          stride = height*1.2;
//        }
//        distance+=stride*steps_2;
//        speed_2 = steps_2 * stride / 2;
//        if(steps_2){
//          cal_2 = speed_2 * weight / 400;
//        }
//        else{
//          cal_2 = weight / 1800;
//        }
//        cal_sum += cal_2;
//        speed_sum += speed_2;
//        speed_avg = speed_sum / cnt_2;
//        // lcd.setCursor(0,1);
//        // lcd.print(distance);
//        steps_2=0;
//        cnt_2++;
//       }
    }
}

void runALgo()
{
  getAccData();
  for(int i = 0;i < 3;++i)
    InternalRunAlgo(i);
    
  if(!(idx_sample[0] % 50))
  {
    int idx = 0;
    for(int i = 1;i < 3;++i)
    {
      if(max_min_diff[i] > max_min_diff[idx])
        idx = i;
    }
    steps += st[idx];
  }
}

void InternalRunAlgo(int axis)
{
  ++idx_sample[axis];
  for(int i = 1;i < MOVING_SIZE;++i)
    moving[axis][i - 1] = moving[axis][i];
  
  moving[axis][MOVING_SIZE - 1] = pos[axis];
  if(abs(moving[axis][MOVING_SIZE - 1] - moving[axis][MOVING_SIZE - 2]) < NOW_PREV_DIFF)
    moving[axis][MOVING_SIZE - 1] = moving[axis][MOVING_SIZE - 2];  
  
  long sum = 0;
  for(int i = 0;i < MOVING_SIZE;++i)
    sum += moving[axis][i];
    
  sum /= MOVING_SIZE;
  for(int i = 1;i < SAMPLE_SIZE;++i)
    samples[axis][i - 1] = samples[axis][i];
  
  samples[axis][SAMPLE_SIZE - 1] = sum;
  if(!(idx_sample[axis] % 50))
  {
    st[axis] = 0;
    int ma = -1e9 , mi = 1e9;
    for(int i = 0;i < SAMPLE_SIZE;++i)
    {
      ma = max(ma , samples[axis][i]);
      mi = min(mi , samples[axis][i]);
    }
    max_min_diff[axis] = ma - mi;
    int threshold = (ma + mi) / 2;
    for(int i = 1;i < SAMPLE_SIZE;++i)
    {
      if(samples[axis][i - 1] >= threshold && samples[axis][i] <= threshold && ma - mi > MAX_MIN_DIFF)
        ++st[axis];
    }
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

int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
 Serial.println(adc_key_in);
  if (adc_key_in > 1000){
     button_pressed =15;
     return btnNONE;
  }
 
  if (abs(button_pressed-adc_key_in)<=10) return btnNONE;

  if (adc_key_in < 50){   
    button_pressed = adc_key_in; 
    return btnRIGHT;
    }  
  if (adc_key_in < 150){  
    button_pressed = adc_key_in; 
    return btnUP;
    } 
  if (adc_key_in < 300){  
    button_pressed = adc_key_in; 
    return btnDOWN;
    } 
  if (adc_key_in < 450){  
    button_pressed = adc_key_in; 
    return btnLEFT;
    } 
  if (adc_key_in < 850){
    button_pressed = adc_key_in; 
    return btnSELECT;
    }     
  return btnNONE;  // when all others fail, return this...
}

void print_to_lCD()
{
  if(state == settings ){
    if(Spage == 0){
      lcd.setCursor(3,0);
      lcd.print(" Settings  ");
      lcd.setCursor(0,1);
      lcd.print("height : ");
      lcd.print(height);
    }
    else if(Spage == 1){
      lcd.setCursor(3,0);
      lcd.print(" Settings  ");
      lcd.setCursor(0,1);
      lcd.print("weight : ");
      lcd.print(weight);
    }
  }
  else{
    if(Mpage == 0){
      lcd.setCursor(3,0);
      lcd.print("Measurement");
      lcd.setCursor(0,1);
      lcd.print("steps : ");
      lcd.print(steps);
    }
    else if(Mpage == 1){
      lcd.setCursor(3,0);
      lcd.print("Measurement");
      lcd.setCursor(0,1);
      lcd.print("distance : ");
      lcd.print(distance);
    }
    else if(Mpage == 2){
      lcd.setCursor(3,0);
      lcd.print("Measurement");
      lcd.setCursor(0,1);
      lcd.print("cals : ");
      lcd.print(cal);
    }
  }
}

void clear_lcd(){
  lcd.setCursor(0,1);
  lcd.print("                ");
}

void process_input(int input){
  switch(input){
    case btnRIGHT:
      if(state == settings){
        Spage = (Spage+1)%2;
      }
      else{
        Mpage = (Mpage+1)%3;
      }
      clear_lcd();
      break;
    case btnUP:
      if(state == settings){
        if(Spage == 0){
          height = min(height+0.1, 2.50);
        }
        else{
          weight = min(weight+5, 200);
        }
      }
      break;
    case btnDOWN:
      if(state == settings){
        if(Spage == 0){
          height = max(height-0.1, 0.50);
        }
        else{
          weight = max(weight-5, 30);
        }
        lcd.setCursor(9, 1);
        lcd.print("    ");
      }
      break;
    case btnLEFT:
      if(state == settings){
        Spage = (Spage-1+2)%2;
      }
      else{
        Mpage = (Mpage-1+3)%3;
      }
      clear_lcd();
      break;
    case btnSELECT:
      state =1;
//      state = (state == settings?1:settings);
      lcd.clear();
      break;
  }
}


