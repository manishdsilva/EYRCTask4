#include <Arduino.h>



/*Team ID: 336
 * Team Members: Manish Dsilva, Amogh Zare, Pritam Mane, Kimaya Desai
 * 
 */
 
 
#define MagF            2                     // electromagnet pin
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

#define encodPinR1      2                       // encoder A pin
#define encodPinR2      3                       // encoder B pin
#define encodPinL1      18                       // encoder A pin
#define encodPinL2      19                       // encoder B pin

#define LOOPTIME        100                     // PID loop time
#define FORWARD         1                       // direction of rotation
#define BACKWARD        2                       // direction of rotation

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
volatile int count_r = 0;                                 // right rotation counter
volatile int countInit_r;
volatile int count_l = 0;                                 // left rotation counter
volatile int countInit_l;
volatile int tickNumber = 0;
volatile boolean run_r = false;                                     // motor moves
volatile boolean run_l = false; 

int VRx = A0;
int VRy = A1;
int SW = 2;

int xPosition = 0;
int yPosition = 0;
int mapX = 0;
int mapY = 0;

void setup() {
 Serial.begin(9600);
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
 
 attachInterrupt(1, rencoder, FALLING);               // arduino pin 3
 attachInterrupt(4, lencoder, FALLING);               // arduino pin 21

 pinMode(VRx, INPUT);
 pinMode(VRy, INPUT);
}

void loop() 
{
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  Serial.println(yPosition);
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);
  if(mapY<-450)
  {
    moveMotor(FORWARD,  200, 1*2); 
  }
  
  else if(mapY>100)
  {   
    moveMotor(BACKWARD, 200, 1*2);   
  }

 delay(10);
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
 
}

void rencoder()  
{                                    // pulse and direction, direct port reading to save cycles
 if (PINB & 0b00000001)    count_r++;                  // if(digitalRead(encodPinB1)==HIGH)   count_r ++;
 else                      count_r--;                  // if (digitalRead(encodPinB1)==LOW)   count_r --;
 if(run_r) 
   if((abs(abs(count_r)-abs(countInit_r))) >= tickNumber)      motorBrake_R();
}

void lencoder()  
{                                    // pulse and direction, direct port reading to save cycles
 if (PINB & 0b00000001)    count_l++;                  // if(digitalRead(encodPinB1)==HIGH)   count_l ++;
 else                      count_l--;                  // if (digitalRead(encodPinB1)==LOW)   count_l --;
 if(run_l) 
   if((abs(abs(count_l)-abs(countInit_l))) >= tickNumber)      motorBrake_L();
}

void motorForward_R(int PWM_val)  {
 analogWrite(PWMR, PWM_val);
 digitalWrite(InR1, LOW);
 digitalWrite(InR2, HIGH);
 run_r = true;
}

void motorForward_L(int PWM_val)  {
 analogWrite(PWML, PWM_val);
 digitalWrite(InL1, LOW);
 digitalWrite(InL2, HIGH);
 run_l = true;
}

void motorBackward_R(int PWM_val)  {
 analogWrite(PWMR, PWM_val);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, LOW);
 run_r = true;
}

void motorBackward_L(int PWM_val)  {
 analogWrite(PWML, PWM_val);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, LOW);
 run_l = true;
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
