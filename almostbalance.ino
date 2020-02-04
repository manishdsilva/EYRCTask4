//#include <Arduino.h>
//#include <SoftwareSerial.h>
#ifdef abs
#undef abs
#endif
float init_time;
float dt2=20;
#define abs(x) ((x)>0?(x):-(x))
//#define rxPin 0
//#define txPin 1

#define yaxUpperThreshold  700  
#define yaxLowerThreshold  700 

#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

#define PI 3.1415926535897932384626433832795
int ax, ay, az;
int gx, gy, gz;
float yax, yay, yaz, ygx, ygy, ygz;
int f_cut = 5;
int n = 1;
float gxPrevious,gyPrevious,gzPrevious;
float ygx_prev, ygy_prev , ygz_prev , yax_prev ,yay_prev ,yaz_prev;
float roll,roll_prev,pitch,pitch_prev,yaw,omega;
float prev_lqr_torque,lqr_torque,torque;
float I_b = 0.0048167;
float x,x_dot,prev_x;
float theta,theta_dot,prev_theta;
//float k1 = -1.0000, k2=-14.0935, k3=120.5796, k4=9.2750; // -1.0000  -32.4753   69.8210    4.8426
//float k1=0.00000, k2=0,k3 = -50.58372,k4=    -1.42088; //float k1=1.00000, k2=0.58314,k3 = -105.58372,k4=    -7.42088;
//float k1=1.00000, k2=0.94, k3 = -66.60, k4= -5.700;
//float k1=0.91 ,   k2 = 0.08,k3= -62.5   ,k4=-15.398523; //float k1=0.91 ,   k2 = 0.08,k3= -62.575181   ,k4=-5.398523;
float k1 = 0.88825,    k2 = 0.12648,  k3 = -62.65876,   k4 = -5.47278;   // for q as identity matrix
 //1.00000     0.51360  -105.28663    -7.34284
//float k1=-1.00000, k2=5.58,k3 = -15,k4= - 80.42088;
//float k1 = 31.623,   k2= 36.828, k3= -271.756,k4 =  -33.752;
//float k1 = 100.00, k2 =   109.09, k3= -1160.62,k4 =   -317.17;
//float k1 = 31.6228,k2=    12.8456,k3=  -134.2097,k4=    -8.2903;
float prev_time; //= millis();
float radius = 0.05, oneRevTicks = 270.0;
#define OUTPUT_READABLE_ACCELGYRO

//SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);


/*Team ID: 336
 * Team Members: Manish Dsilva, Amogh Zare, Pritam Mane, Kimaya Desai
 * 
 */
 //Macros
#define MagF            50                     // electromagnet pin
#define buzz_pin        31                     // Buzzer pin
#define RED_pin         43                     //LED Pin
#define Common_pin      45                     //LED Pin
#define GREEN_pin       47                     //LED Pin
#define BLUE_pin        49                     //LED Pin
#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  
#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

//Global Variables
int received;
int analogX;
int analogY;

#define encodPinR1      2                       // encoder A pin
#define encodPinR2      3                       // encoder B pin
#define encodPinL1      18                       // encoder A pin
#define encodPinL2      19                       // encoder B pin

#define LOOPTIME        100                     // PID loop time
#define FORWARD         1                       // direction of rotation
#define BACKWARD        2                       // direction of rotation
#define RIGHTWARD       3                       // direction of rotation
#define LEFTWARD        4                       // direction of rotation

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
volatile int count_r = 0;                                 // right rotation counter
volatile int countInit_r;
volatile int count_l = 0;                                 // left rotation counter
volatile int countInit_l;
volatile int tickNumber = 0;
volatile boolean run_r = false;                                     // motor moves
volatile boolean run_l = false; 


/*
 * motor_init(): To initialize motor pins
 * MAG_init(): To initialize Electromagnet Pins
 * LED_init(): To initialize the LED pins
 * BUZZ_init(): Buzzer initialization
 */
void setup() {
 Serial.begin(9600);
 motor_init();
 MAG_init(); 
 LED_init();
 BUZZ_init();
 accel_init();
 //pinMode(rxPin, INPUT);
 //pinMode(txPin, OUTPUT);
 //xbee.begin(19200);
 init_time = millis();
}

/*
 * Input: None
 * Output: None
 * Description: accelerometer Initialization
 * parameters: None
 */
void accel_init()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
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

/*
 * Input: None
 * Output: None
 * Description: Buzzer Initialization
 * parameters: None
 */
void BUZZ_init(){
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Motor Initialization
 * parameters: None
 */
void motor_init(void)
{
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);
 
 pinMode(encodPinR1, INPUT);
 pinMode(encodPinR2, INPUT);
 pinMode(encodPinL1, INPUT);
 pinMode(encodPinL2, INPUT);
 
 digitalWrite(encodPinR1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinR2, HIGH);
 digitalWrite(encodPinL1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinL2, HIGH);
 
 attachInterrupt(1, rencoder, RISING);               // arduino pin 3
 attachInterrupt(4, lencoder, RISING);               // arduino pin 21
}

void moveMotor(int direction, int PWM_val, long tick)  
{
 count_r = 0;
 count_l = 0;
 countInit_r = count_r;    // abs(count)
 countInit_l = count_l;
 tickNumber = tick;
 if(direction==FORWARD)          
 {  
  motorForward_R(PWM_val);
  motorForward_L(PWM_val);
 }
 else if(direction==BACKWARD)    
 { 
  //Serial.println("Backward");
  motorBackward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==RIGHTWARD)          
 {  
  motorForward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==LEFTWARD)          
 {  
  motorBackward_R(PWM_val);
  motorForward_L(PWM_val);
 }
 
}

void rencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinR1)==HIGH)   count_r ++;  //motors forward ticks++              
 else if (digitalRead(encodPinR2)==HIGH)   count_r --; //motors backward ticks--   
 if(run_r) 
   if((abs(abs(count_r)-abs(countInit_r))) >= tickNumber)      motorBrake_R();
}

void lencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinL1)==HIGH)   count_l ++;  //motors forward ticks++              
 else if (digitalRead(encodPinL2)==HIGH)   count_l --; //motors backward ticks--   
 if(run_l) 
   if((abs(abs(count_l)-abs(countInit_l))) >= tickNumber)      motorBrake_L();
}


void motorForward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Forward Motion of Right Motor
 * parameters: PWM value
 */
void motorForward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
  
}


/*
 * Input: None
 * Output: None
 * Description: Backward Motion of Left Motor
 * parameters: PWM value
 */
void motorBackward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}


/*
 * Input: None
 * Output: None
 * Description: Backward Motion of Right Motor
 * parameters: PWM value
 */
void motorBackward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}


void motorBrake()  {
 analogWrite(PWMR, 0);
 analogWrite(PWML, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_r = false;
 run_l = false;
}

void motorBrake_R()  
{
 analogWrite(PWMR, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 run_r = false;
}

void motorBrake_L()  
{
 analogWrite(PWML, 0);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_l = false;
}

/*
 * Input: None
 * Output: None
 * Description: Electromagnet Initialization
 * parameters: None
 */
void MAG_init(void)
{
    pinMode(MagF, OUTPUT);
    digitalWrite(MagF, LOW);
}

/*
 * Input: None
 * Output: None
 * Description: Magnet Pickup
 * parameters: None
 */
void MagPick(void)  
{
    digitalWrite(MagF, HIGH);
}

/*
 * Input: None
 * Output: None
 * Description: Magnet Drop
 * parameters: None
 */
void MagDrop(void)  
{
    digitalWrite(MagF, LOW);
}


void lowpassfilter(float ax,float ay,float az,int16_t n,int16_t f_cut)
{
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line

  if(n == 1)
  {
    yax = (1-alpha)*ax ;
    yay = (1-alpha)*ay ;
    yaz = (1-alpha)*az ;
  }
  else
  {
    yax = (1-alpha)*ax + alpha*yax_prev;
    yay = (1-alpha)*ay + alpha*yay_prev;
    yaz = (1-alpha)*az + alpha*yaz_prev;  
  }  
  yax_prev = yax;
  yay_prev = yay;
  yaz_prev = yaz;
    
}

void highpassfilter(float gx,float gy,float gz,int16_t n,int16_t f_cut)
{
  
  float dT = 0.01;  //time in seconds
  float Tau= 1/(2*3.1457*f_cut);                   //f_cut = 5
  float alpha = Tau/(Tau+dT);                //do not change this line
 
  if(n == 1)
  {
    ygx = (1-alpha)*gx ;
    ygy = (1-alpha)*gy ;
    ygz = (1-alpha)*gz ;

  }
  else
  {
    ygx = (1-alpha)*ygx_prev + (1-alpha)*(gx - gxPrevious);
    ygy = (1-alpha)*ygy_prev + (1-alpha)*(gy - gyPrevious);
    ygz = (1-alpha)*ygz_prev + (1-alpha)*(gz - gzPrevious);
  }
  gxPrevious = gx;
  gyPrevious = gy;
  gzPrevious = gz;

  ygx_prev = ygx;
  ygy_prev = ygy;
  ygz_prev = ygz;
  
}

void comp_filter_pitch(float ax,float ay,float az,float gx,float gy,float gz)
{
  float alpha = 0.03;
  float dt = 0.01;

  if (n==1)
  {
    pitch = (1-alpha)*((-1)*gx*dt) + alpha*(atan(ay/abs(az))*180/PI); 
  }
  else
  {
    pitch = (1-alpha)*(pitch_prev - (gx*dt)) + alpha*(atan(ay/abs(az))*180/PI);
  }
  pitch_prev=pitch;
}

void comp_filter_roll(float ax,float ay,float az,float gx,float gy,float gz)
{
  float alpha = 0.03;
  float dt = 0.01;

  if (n==1)
  {
    roll = (1-alpha)*((-1)*gy*dt) + alpha*(atan(ax/abs(az))*180/PI);
  }
  else
  {
    roll = (1-alpha)*(roll_prev - (gy*dt)) + alpha*(atan(ax/abs(az))*180/PI);
  }
  roll_prev=roll;  
}
void motorControl(int torque)
{
 //torque between 0-255
 if (torque >= 0) 
 { // drive motors forward
 
 torque = abs(torque);
 motorForward_R(torque); 
 motorForward_L(torque); 
}
 else{ 
 // drive motors backward
 torque = abs(torque);
 motorBackward_R(torque); 
 motorBackward_L(torque);
 }
}
void loop()
{
  
  //MPU
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  lowpassfilter(ax,ay,az,n,f_cut);
  highpassfilter(gx,gy,gz,n,f_cut);
  comp_filter_pitch(yax,yay,yaz,ygx,ygy,ygz);
  comp_filter_roll(yax,yay,yaz,ygx,ygy,ygz);
  //MPU ENDS

  


  
  //STATE VARIABLES
  int flag = -1;
  if((yax-yaxUpperThreshold)>0)
    flag = 1;
  theta = flag*abs(roll-2);
  theta_dot = (theta-prev_theta)/dt2;
  x = (count_r*2*PI*radius)/270; //displacement
  x_dot = (x - prev_x)/dt2;//linear velocity
  //STATE VARIABLES ENDS

  

  //PID
  int pid_torque = 50*theta+130*(theta-prev_theta);//+ 20*x+0*(x-prev_x); //50 450 //20-20 good
  //Serial.println(50*theta+10*(theta-prev_theta));
  pid_torque = constrain(pid_torque, -150, 150);
  //PID ENDS
  Serial.print(30*theta);Serial.print("\t");Serial.println(10*theta-prev_theta);
  lqr_torque =  -((x*k1)+(x_dot*k2)+(theta*k3)+(theta_dot*k4))*(255/255);
  lqr_torque = constrain(lqr_torque, -200, 200);
  //Serial.print((-lqr_torque)); Serial.println();
  
  //Serial.print(-x*k1);Serial.print("\t");Serial.print(-x_dot*k2);Serial.print("\t");Serial.print(-theta*k3);Serial.print("\t");Serial.println(-theta_dot*k4);
  //PREVIOUS STATE VARIABLES
  prev_theta = theta;
  prev_x = x;
  //PREVIOUS STATE VARIABLES ENDS
  
  //Serial.print(abs(lqr_torque)); Serial.println();
  //Serial.println(constrain(pid_torque,-100,100));

  //Serial.println(theta);
  //Serial.println(theta_dot);
  
  //prev_lqr_torque = lqr_torque;
  
  if( (yaxLowerThreshold<yax) && (yax<yaxUpperThreshold))
  {
    //Serial.print("Stable in x direction");Serial.print("\t");
  }
  motorControl(-pid_torque);
  /*
  else if(yax>yaxUpperThreshold)
  {
    //Serial.print("Falling Backward");
    motorControl(-abs(lqr_torque));
  }
  else if(yax<yaxLowerThreshold)
  {
    //Serial.print("Falling Forward");Serial.print("\t");
    motorControl(abs(lqr_torque));
  }
  */
  //Serial.println();
  
  //Accept only if characters are 18 or more
  
  if(Serial.available()>=18)                                
  {
    //0x7E is the first byte of Xbee frame
    //Accept data starting with 0X7E
    if(Serial.read()==0x7E)
    {
      
      //Ignore the first 11 bytes of data
      for(int i=1 ; i < 11 ; i++)
      {
        byte discardByte = Serial.read();    
      }

      //Accept the data from the 11th byte to 16th byte
      byte digitalMSB = Serial.read();
      byte digitalLSB = Serial.read();
      byte analogMSB1 = Serial.read();
      byte analogLSB1 = Serial.read();
      byte analogMSB2 = Serial.read();
      byte analogLSB2 = Serial.read();

      //Combining LSB and MSB by left shifting the MSB and adding it to the LSB
      int analogX = analogLSB1+ analogMSB1*256;
      int analogY = analogLSB2+ analogMSB2*256;
      
      //Serial.print(analogX);
      //Serial.print("\t");
      //Serial.println(analogY);

      int else_flag = 0;

      //Turn On the LED if the digitalMSB value is 0x01
      if(digitalMSB == 0x01) //led switch
      {
        digitalWrite(RED_pin, LOW);
        digitalWrite(GREEN_pin, HIGH); 
        digitalWrite(BLUE_pin, HIGH); 
        digitalWrite(MagF, LOW);
        digitalWrite(buzz_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Magnet if the digitalLSB value is 0x04
      if(digitalLSB == 0x04) //magnet switch
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(buzz_pin, HIGH);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Buzzer if the digitalLSB value is 0x08
      else if(digitalLSB == 0x08) //buzzer switch
      {
        digitalWrite(buzz_pin, LOW);
        digitalWrite(MagF, LOW);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the Magnet and Buzzer if the digitalLSB value is 0x0C
      else if(digitalLSB == 0x0C) //magnet and buzzer both
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(buzz_pin, LOW);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn off everything
      else if(else_flag == 0)
      {
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);    
        digitalWrite(MagF, LOW);
        digitalWrite(buzz_pin, HIGH);
      }
      
      //No Motion
      //Serial.print(analogX);Serial.print("\t");Serial.println(analogY);
      if((analogX > 1023) || (analogY > 1023 ))
        int a;
  
      //Both Wheels Forward
      else if((analogX > 900)  && (400 < analogY) && (analogY <800 ))
      {
        Serial.println("Both Wheels Forward");
        moveMotor(FORWARD,  200, 3*2);
      }
  
      //Both Wheels Backward
      else if((analogX > 900) && (analogY < 300) )
      {
        Serial.print("LSB ");
        Serial.println(digitalLSB);
        Serial.println(digitalMSB);
        moveMotor(BACKWARD,  200, 3*2);
      }
  
      //Left Wheel Backward, Right Wheel Forward
      else if((400< analogX)&&(analogX <800 )&& (analogY > 900) )
      {
        Serial.println("left wheel Backward, right wheel Forward");
        moveMotor(RIGHTWARD,  200, 3*2);
      }
  
      //Left Wheel Forward, Right Wheel Backward
      else if((analogX < 300) && (analogY > 900))
      {
        Serial.println("left wheel Forward, right wheel Backward");
        moveMotor(LEFTWARD,  200, 3*2);
      }

    else
    {
      motorBrake();
    }
    
   } 
  } 
  if(n!=1)
  {
  dt2 = (millis()-init_time) / 1000;
  init_time = millis();
  }
  //Serial.println(dt2*1000);
  delay(7); 
  n++;
  
  
}
