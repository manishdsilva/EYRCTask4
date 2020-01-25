#include <Arduino.h>



/*Team ID: 336
 * Team Members: Manish Dsilva, Amogh Zare, Pritam Mane, Kimaya Desai
 * 
 */
 //Macros
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

//Global Variables
int received;
int analogX;
int analogY;

/*
 * motor_init(): To initialize motor pins
 * MAG_init(): To initialize Electromagnet Pins
 * LED_init(): To initialize the LED pins
 * BUZZ_init(): Buzzer initialization
 */
void setup() {
   
 motor_init();
 MAG_init(); 
 LED_init();
 BUZZ_init();
 Serial.begin(9600);
     
}

/*
 * Input: None
 * Output: None
 * Description: LED Initialization
 * parameters: None
 */
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
    //pinMode(MagF, OUTPUT);
    
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

/*
 * Input: None
 * Output: None
 * Description: Forward Motion of Left Motor
 * parameters: PWM value
 */
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


/*
 * Input: None
 * Output: None
 * Description: Forward Motion of Both Motors
 * parameters: None
 */
void bothForward()
{
    motorForward_R(200);
    motorForward_L(200);  
}

/*
 * Input: None
 * Output: None
 * Description: Backward Motion of Both Motors
 * parameters: None
 */
void bothBackward()
{
    motorBackward_R(200);
    motorBackward_L(200);  
}

/*
 * Input: None
 * Output: None
 * Description: Left Motor Forward and Right Motor Backward
 * parameters: None
 */
void LForwardRBackward()
{
  motorBackward_R(200);
  motorForward_L(200);  
        
}

/*
 * Input: None
 * Output: None
 * Description: Left Motor Backward and Right Motor Forward
 * parameters: None
 */
void LBackwardRForward()
{

  motorForward_R(200);
  motorBackward_L(200);  
           
}

/*
 * Input: None
 * Output: None
 * Description: Stopping Both Motors
 * parameters: None
 */
void motorsStop(void)
{
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, LOW);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
    
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


/*
 *
 */
void loop()
{

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
      
      Serial.print(analogX);
      Serial.print("\t");
      Serial.println(analogY);

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
    if((analogX > 1023) || (analogY > 1023 ))
      int a;

    //Both Wheels Forward
    else if((analogX > 900)  && (400 < analogY) && (analogY <800 ))
    {
      Serial.println("Both Wheels Forward");
      bothForward();
    }

    //Both Wheels Backward
    else if((analogX > 900) && (analogY < 300) )
    {
      Serial.println("Both wheels Backward");
      bothBackward();
    }

    //Left Wheel Backward, Right Wheel Forward
    else if((400< analogX)&&(analogX <800 )&& (analogY > 900) )
    {
      Serial.println("left wheel Backward, right wheel Forward");
      LBackwardRForward();
    }

    //Left Wheel Forward, Right Wheel Backward
    else if((analogX < 300) && (analogY > 900))
    {
      Serial.println("left wheel Forward, right wheel Backward");
      LForwardRBackward();
    }
    else
    {
      motorsStop();
    }
   
   }
  //Serial.flush();
  
  }
  
}
