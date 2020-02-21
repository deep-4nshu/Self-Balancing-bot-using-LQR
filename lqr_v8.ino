#include "I2Cdev.h"
//#include "MPU6050.h"
#include "math.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include <digitalWriteFast.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MagF            51                      // electromagnet pin
#define buzz_pin        31                      // electromagnet pin
#define RED_pin         43                      //LED Pin
#define Common_pin      45                      //LED Pin
#define GREEN_pin       47                      //LED Pin
#define BLUE_pin        49                      //LED Pin
#define InL1            11                      // motor pin
#define PWML            46                     // PWM motor pin
#define InL2            12                       // motor pin
#define InR1            7                       // motor pin
#define PWMR            45                       // PWM motor pin
#define InR2            8                       // motor pin

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
double input;int time_delay;
#define enc2pinA 3  
#define enc2pinB 4  
//
float f_cut=10.50;
float alpha1=0.01250;
MPU6050 mpu;
// Global Variables
float slope_offset=0, move_offset=0, max_angle_vel=4, max_angle_enc=2;
volatile float accel_angle=0, gyro_angle=0;
volatile float rotation_left=0, rotation_right=0;
volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;
unsigned long last_task_time_PID=0;
volatile float y_setpoint=0;
int now_time=0,previous_time=0;
//timer 4

//encoder variables
volatile int right_count=0;
volatile int newposition_1=0,newposition_2=0;
volatile int oldposition_1=0,oldposition_2=0;
unsigned long timenew,timeold=0;
float vel =0, velb=0,vela = 0,ul=0, ur=0,v0=0,v1=0;

//MPU6050 accelgyro;
int flagr=0,flagl=0;
float x; //state variables position using encoder
float x_dot;//state variable velocity
float theta;// angle using MPU6050
float theta_dot;// angular velocity
float pitch_angle=0;// old pitch angle

//sensor values
int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;
float old1=0, old2=0 ,old3=0;
float AX1=0,AY1=0,AZ1=0;
float old_gx=0, new_gx =0,old_gy=0 ,new_gy=0 ,old_gz=0 ,new_gz=0;
float GX_N=0 ,GY_N=0, GZ_N=0;
float GX_O=0, GY_O=0, GZ_O=0;
float old_pitch_angle=0, new_pitch_angle=0;
float pitch_acc=0,  pitch_gyro=0;
//volatile int32_t right_count=0,left_count=0,old_encoder=0; //for counting interrupts by both encoders
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
float k1,k2,k3,k4;//define your K matrix here calculated using octave or matlab
float u=0; //input for the system

// Structure to handle the various inputs from the controller
void setup(){
    Serial.begin(9600);
    Serial3.begin(9600);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
     mpu.initialize();
 devStatus = mpu.dmpInitialize();
 // supply your own gyro offsets here, scaled for min sensitivity
 /*mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
*/
 // make sure it worked (returns 0 if so)
 if (devStatus == 0){
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);
attach_interrupt();
    motor_init();
   mpuIntStatus = mpu.getIntStatus();  
dmpReady = true;
packetSize = mpu.dmpGetFIFOPacketSize();
 }
}   
void loop(){
sensor_data();  
}
void sensor_data(){
while (!mpuInterrupt && fifoCount < packetSize){
  //now_time=millis();
  new_pitch_angle=input;
  lqr();
 }
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();

 // get current FIFO count
 fifoCount = mpu.getFIFOCount();

 if ((mpuIntStatus & 0x10) || fifoCount == 1024){
 mpu.resetFIFO();
 }
 else if (mpuIntStatus & 0x02){
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

 mpu.getFIFOBytes(fifoBuffer, packetSize);
 
 fifoCount -= packetSize;
 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180/M_PI;
 }
 }

void attach_interrupt(){ //attaching interrups to digital pins 2,3 and also initialising the motor pins  
    pinMode(enc2pinA,INPUT);
    pinMode(enc2pinB,INPUT);
    digitalWrite(enc2pinA,HIGH);
    digitalWrite(enc2pinB,HIGH);
    attachInterrupt(1, right_movement,FALLING);
    attachInterrupt(0, dmpDataReady, RISING);
    right_count = 0;
}

void dmpDataReady(){
 mpuInterrupt = true;
}

void right_movement(){ // ISR for right encoder's motor
    int state = digitalRead(enc2pinA);
    if(digitalRead(enc2pinB))
    state ? right_count-- : right_count++;
    else
    state ? right_count++ : right_count--;
}

void motor_init(){
    pinMode(MagF, OUTPUT);
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void motorForwardL(int PWM_val)  {
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
    flagl=1;
}
void motorBackwardL(int PWM_val)  {
    analogWrite(PWML, PWM_val);
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, HIGH);
    flagl=0;
}
void motorBackwardR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR2, LOW);
    digitalWrite(InR1, HIGH);
}
void motorForwardR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
void stop_motorR(){
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}
void stop_motorL(){
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, LOW);  
}

void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);
    digitalWrite(RED_pin, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(GREEN_pin, HIGH);
    digitalWrite(BLUE_pin, HIGH);
}
void BUZZ_init(){
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}
void MAG_init(){
    pinMode(MagF, OUTPUT);
    digitalWrite(MagF, LOW);
}
void MagPick(void)  {
    digitalWrite(MagF, HIGH);
}
void MagDrop(void)  {
    digitalWrite(MagF, LOW);
}

//timer 3 FOR SENSOR sampling
void balance_using_motor(float input_1, float input_2){//this function takes u as input maps it and depending on its sign moves motors in particular directions and adjust their speed
 
  uint8_t VR=(constrain((abs(input_1)), 0, 125));
  uint8_t VL=(constrain((abs(input_2)), 0,125));
  if(input_1>0){
   motorForwardR(VL);
   motorForwardL(VR);
}
 else if(input_1<0){
   motorBackwardL(VR);
   motorBackwardR(VL);   
 }
}

void process_encoder()
{
   v1 = 1*(((newposition_2-oldposition_2)/(time_delay*270))*(2*3.14));
   vel = (v1);
   vela=vela +vel*time_delay;
}

void lqr(){ // this function takes values of encoders and mpu6050 sensor as x and theta and solves equation u=-kx and pass value of u as pwm for motor controlling
  time_delay=now_time-previous_time;
  theta=new_pitch_angle;
  theta_dot= ((new_pitch_angle-pitch_angle)/time_delay);
  pitch_angle=new_pitch_angle;
  y_setpoint=0;
  newposition_2 = right_count;
  process_encoder();
  oldposition_2=newposition_2;
  previous_time=now_time;
  if((theta)>=60|| (theta)<=-60)
 { 
  ul=2;
  ur=2;
 }
 else
 {
  ul=(((vela*(k1)) + (vel*(k2))+((theta)*(k3))+(theta_dot*(k4))));
 }
  now_time=millis();
  balance_using_motor(ul,ul);
 }
