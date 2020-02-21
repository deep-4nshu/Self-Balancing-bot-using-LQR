#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
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
#define enc1pinA 2 
#define enc1pinB 5
#define enc2pinA 3  
#define enc2pinB 4  
//
float f_cut=10.50;
float alpha1=0.01250;
int last_sensor_task,last_lqr_task;
int flagright=0, flagleft=0,flagforward=0, flagbackward=0, flagstop=0;
int discardByte[22];
int n=1;
int A[10],B[10];
int a,b;

// Global Variables
float slope_offset=0, move_offset=0, max_angle_vel=4, max_angle_enc=2;
volatile float accel_angle=0, gyro_angle=0;
volatile float rotation_left=0, rotation_right=0;
volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;
unsigned long last_task_time_PID=0;
volatile float y_setpoint=0;

//timer 4
volatile unsigned long int time_ms = 0;
volatile unsigned long int time_sec = 0;
unsigned long elapsed_time;
volatile unsigned long int sensor_ms = 0;

//encoder variables
volatile int right_count=0;
volatile int left_count=0;
volatile int newposition_1=0,newposition_2=0;
volatile int oldposition_1=0,oldposition_2=0;
unsigned long timenew,timeold=0;
float vel =0, velb=0,vela = 0,ul=0, ur=0,v0=0,v1=0;

MPU6050 accelgyro;
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

float u=0; //input for the system

void setup()

{   Serial.begin(9600);
    Serial3.begin(9600);
    timer3_init();  // for sensor reading gy87
    timer4_init();  // for calculating overall program time epoch()
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    attach_interrupt();
    motor_init();
    LED_init();
    BUZZ_init();
    MAG_init();
    accelgyro.initialize();
    start_timer3(); // for sampling sensor data @0.005 sec
    start_timer4(); // for controlling motors  @0.005 sec
    //start_timer1();
    
}   
void loop()
{
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
   task(); //for every task
}

void task()
{
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
if((sensor_time()- last_sensor_task)>= 1)
{
    last_sensor_task = sensor_time();
    process_sensor();
    //Serial.println("sensor");
}
if((epoch()- last_lqr_task)>= 1)
{
    //Serial.println("epoch");
    last_lqr_task = epoch();
    lqr(); 
    //for calculating the lqr 
    }
}

void attach_interrupt() //attaching interrups to digital pins 2,3 and also initialising the motor pins
{  
    pinMode(enc1pinA,INPUT);
    pinMode(enc1pinB,INPUT);
    pinMode(enc2pinA,INPUT);
    pinMode(enc2pinB,INPUT);
    digitalWrite(enc1pinA,HIGH);
    digitalWrite(enc1pinB,HIGH);
    digitalWrite(enc2pinA,HIGH);
    digitalWrite(enc2pinB,HIGH);
    attachInterrupt(1, right_movement,FALLING);
    attachInterrupt(0, left_movement, FALLING);
    right_count = 0;
    left_count = 0;
}

void left_movement() //ISR for left motor's encoder
{
    int state = digitalRead(enc1pinA);
    //Serial.println(state);
    if(digitalRead(enc1pinB))
    state ? left_count-- : left_count++;
    else
    state ? left_count++ : left_count--;
    }

void right_movement() // ISR for right encoder's motor
{
    int state = digitalRead(enc2pinA);
    if(digitalRead(enc2pinB))
    state ? right_count-- : right_count++;
    else
    state ? right_count++ : right_count--;
}

void motor_init()
{
    pinMode(MagF, OUTPUT);
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void motorForwardL(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
    flagl=1;
}
void motorBackwardL(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL2, LOW);
    digitalWrite(InL1, HIGH);
    flagl=0;
}
void motorBackwardR(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR2, LOW);
    digitalWrite(InR1, HIGH);
}
void motorForwardR(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}
void stop_motorR()
{
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}
void stop_motorL()
{
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
void BUZZ_init()
{
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
void timer3_init()
{
    TCCR3B = 0x00;  // Stop Timer
    TCNT3 = 0xFEC7; //0.005 sec
    OCR3A = 0x0000; // Output Compare Register (OCR) - Not used
    OCR3B = 0x0000; // Output Compare Register (OCR) - Not used
    OCR3C = 0x0000; // Output Compare Register (OCR) - Not used
    ICR3 = 0x0000;  // Input Capture Register (ICR)  - Not used
    TCCR3A = 0x00;
    TCCR3C = 0x00;
}
void start_timer3()
{
    TCCR3B = 0x04; // Prescaler 256
    TIMSK3 = 0x01; // Enable Timer Overflow Interrupt
}
ISR(TIMER3_OVF_vect) // overflow flag is raised when TCNT reaches 65536
{
     TIMSK3 = 0x00;
     process_sensor();
     sensor_ms++;      // Increment ms value
     TCNT3 = 0xFEC7;   //0.005 ,256 as pre scalar
    //TCNT3  = 0x0FD87;//0.01 ,256 as pre scalar
    //TCNT3 = 0xFB1E;  //0.02 using ,256 as pre-scalar
      TIMSK3 = 0x01;
}
unsigned int sensor_time()
{
  return sensor_ms;
  }
  
void process_sensor()
{
     low_pass_filter(ax1,ay1,az1) ;  // accelerometer values to low pass filter
     high_pass_filter(gx1,gy1,gz1) ; // gyroscope values to high pass filter
     comp_pitch(AX1,AY1,AZ1,GX_N,GY_N,GZ_N);// comp filter for calculating pitch
     theta=new_pitch_angle;
     theta_dot=((new_pitch_angle-pitch_angle)/0.005);
     pitch_angle=new_pitch_angle;
     }
//timer 4 FOR FULL PROGRAM TIME
void timer4_init()
{
  TCCR4B = 0x00;    // Stop Timer
  TCNT4  = 0x63C0;  // 0.0009999593097s (~0.001s)
  OCR4A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR4B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR4C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR4   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR4A = 0x00;
  TCCR4C = 0x00;
}
void start_timer4()
{
  TCCR4B = 0x02;    // Prescaler 8
  TIMSK4 = 0x01;    // Enable Timer Overflow Interrupt
}

ISR(TIMER4_OVF_vect)
{
  TIMSK4 = 0x00;  // Reload counter value
  time_ms++;      // Increment ms value
   lqr();
   TCNT4=0xD8F0; //DELAY 0F 0.005 SEC 8 as pre scalar
   //TCNT4=0xB1E0;//DELAY 0F 0.01 SEC 8 as pre scalar
   //TCNT4=0x63C0;//DELAY OF 0.02 SEC 8 as pre scalar
 TIMSK4= 0x01;
}
unsigned long epoch()
{
  elapsed_time = time_ms;
  return elapsed_time;
}

void low_pass_filter(int16_t ax,int16_t ay,int16_t az){ //low pass filter for accelerometer data
  float dT = 0.005;
  float Tau= 1/(2*3.1416*f_cut);
  float alpha = Tau/(Tau+dT);//0.031;
  float ax2=(float)ax;
  float ay2=(float)ay;
  float az2=(float)az;
  AX1=(1-alpha)*ax2+alpha*old1;
  old1 = AX1;
  AY1=(1-alpha)*ay2+alpha*old2;
  old2 = AY1;
  AZ1=(1-alpha)*az2+alpha*old3;
  old3 = AZ1;
  AX1 = (AX1)/16384;
  AY1 = (AY1)/16384;
  AZ1 = (AZ1)/16384;
}
void high_pass_filter(int16_t gx,int16_t gy,int16_t gz){ //high pass filter for gyroscope data
  float dT = 0.005;
  float Tau= 1/(2*3.141*f_cut);
  float alpha = Tau/(Tau+dT);
  float gx2=(float)gx;
  float gy2=(float)gy;
  float gz2=(float)gz;
  new_gx = gx;
  GX_N = (1-alpha)*GX_O + (1-alpha)*(new_gx - old_gx);
  old_gx = new_gx;
  GX_O = GX_N;
  new_gy = gy;
  GY_N = (1-alpha)*GY_O + (1-alpha)*(new_gy - old_gy);
  old_gy = new_gy;
   GY_O = GY_N;
  new_gz = gz;
  GZ_N = (1-alpha)*GZ_O + (1-alpha)*(new_gz - old_gz);
  old_gz = new_gz;
  GZ_O = GZ_N;
  GX_N=GX_N/131;
  GY_N=GY_N/131;
  GZ_N=GZ_N/131;
}
void comp_pitch(float ax,float ay,float az,float gx,float gy,float gz){ // this fucntion calculates pitch angle
  pitch_acc =atan2f(-ax,abs(az));
  pitch_acc=(pitch_acc*180)/3.14;
  pitch_gyro =(gy*0.01);
  new_pitch_angle = (1-alpha1)*(old_pitch_angle+pitch_gyro) + (alpha1*pitch_acc);
  old_pitch_angle = new_pitch_angle;
  new_pitch_angle = new_pitch_angle*(-1);
  //new_pitch_angle = (new_pitch_angle)/57*3;
  //Serial.print("pitch= ");
  //Serial.print(new_pitch_angle,6);Serial.println(" ");
}

void balance_using_motor(float input_1, float input_2){//this function takes u as input maps it and depending on its sign moves motors in particular directions and adjust their speed
 
  uint8_t VR=(constrain((abs(input_1)), 10, 180));
  uint8_t VL=(constrain((abs(input_2)), 10,180));
  //Serial.print(" v0= ");Serial.print(v0);
  //Serial.print(" v1= ");Serial.print(v1);
  //Serial.print(" VR= ");Serial.print(VR);
  //Serial.print(" VL= ");Serial.println(VL);
 
 if(input_1>0){
   //VR=VR+10;VL=VL+10;
   motorForwardR(VL);
   //Serial.println("1");   
  motorForwardL(VR);
}
 else if(input_1<0){
  
   motorBackwardL(VR);
   //Serial.println("2");
   motorBackwardR(VL);   
 }
}

void process_encoder()
{
   v0 = (((newposition_1-oldposition_1)/(0.005*270))*(2*3.14));//left count
   v1 = -1*(((newposition_2-oldposition_2)/(0.005*270))*(2*3.14));//right count
   vel = (v0 + v1)*0.5;
   vela=vela +vel*0.005;
   velb=velb +v1*0.02; 
}

void lqr(){ // this function takes values of encoders and mpu6050 sensor as x and theta and solves equation u=-kx and pass value of u as pwm for motor controlling
  y_setpoint=0;
  newposition_1 = left_count;
  newposition_2 = right_count;
  process_encoder();
  oldposition_1=newposition_1;
  oldposition_2=newposition_2;
  vel=0;
  if((theta)>=60|| (theta)<=-60)
 { 
  ul=2;
  ur=2;
 }
 else
  {
  ul=(((vela*(0.2)) + (vel*(1.08))+(theta*(27.52332))+(theta_dot*(7))));
}
  if((theta)>=-0.5 && (theta)<=0.5)
 { 
  ul=2;
  ur=2;
 }
 
  balance_using_motor(ul,ul);
  }
