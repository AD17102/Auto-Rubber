//When done with the testing and confident it will work on it's own,
//remove all the Serial.print() functions from the code. 
//They're under the comments that have "(testing)" in them. 
//Use Ctrl + f to find and remove them 


#include <Servo.h>
#include <Wire.h>
#include <math.h>

//Define pins  for the motor
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8
#define PWM_MOTOR_1 5
#define EN_PIN_1 A0
#define EN_PIN_2 A1

//straight bool for going faster when on a straight path

//ultrasonic
#define trigPin 10
#define ultra_pin1 11
#define ultra_pin2 12


//Temp Global Variables for the gyroscope 
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
#define    GYRO_FULL_SCALE_2000_DPS   0x18
#define    ACC_FULL_SCALE_16_G       0x18

//multiplier for adjusting the speed when going up the ramp 
#define     MULTI 1.5

//IR  SENSOR AND SERVO PINS 
//main reason why one of the pins is A3 is because there is no more digital pins left lol 
#define  SensorPinA A3
#define SensorPinB  6
#define SensorPinC  4
#define SensorPinD  3
#define SensorPinE  2


//SERVO GLOBAL VARS
#define SERVO_PIN 9
Servo myservo;


//INFRARED SENSOR FUNCTION-------------------------------------------------------------------------------------------------------------------------/ 
//STEER FUNCTION - WRITES THE SERVO MOTOR TO STEER THE CAR. USES INFRARED SENSORS TO WRITE SPECIFIC ANGLES
void Steer(short numberOfSensorsOn, short Leftmost, short Left, short Centre, short Right, short Rightmost){

  //Print direction . KEEP IN MIND THAT THERE IS A PRINT STATEMENT IN EACH CASE AND IF STATEMENT (testing)
  Serial.print("\tDIRECTION: ");
  switch(numberOfSensorsOn){
    case 1:      
     if(Rightmost == 1)
     {
        Serial.print("STEEP RIGHT"); 
        myservo.write(120);
     }
     else if(Leftmost == 1)
     {
        Serial.print("STEEP LEFT");    
        myservo.write(60);
        
     }else if (Centre == 1){

       Serial.print("CENTER");    
       myservo.write(90);  
       
     }
     if(Left == 1){
      myservo.write(69); 
      Serial.print("SLIGHT LEFT"); 
     }  
     else if (Right == 1){
       myservo.write(109); 
       Serial.print("SLIGHT RIGHT");
       
     }  
      
      break;

    case 2:
     if(Centre == 1 && Right == 1)
     {
        Serial.print(" RIGHT");    
        myservo.write(94);
        
     }
     else if(Centre == 1 && Left == 1)
     {
       
       Serial.print(" LEFT");    
       myservo.write(86);
       
     }
       break;

    case 3:
    
       Serial.print("STRAIGHT");   
       myservo.write(90);
       
       break;

    default :
    
      Serial.print("NO LINE "); 

       break;
  }

}





//GYROSCOPE SENSOR FUNCTIONS -----------------------------------------------------------------------------------------------------------------------------------------/ 

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data){
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

//ULTRASONIC SENSOR FUNCTIONS ----------------------------------------------------------------------------------------------------------------------------------------/


// READ ULTRASONIC SENSOR FUNCTION. RETURNS DISTANCE
short  Ultrasonic(uint8_t pin){
  digitalWrite(trigPin, LOW);
  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(100); 
  digitalWrite(trigPin, LOW); 
  unsigned duration2 = pulseIn(pin, HIGH, 50000); 
  unsigned short distance = duration2 * 0.034 / 2; 

  return distance; 
}


//OBSTACLE DETECTION FUNCTION. RETURNS TRUE IF THERE IS AN OBJECT INFRONT
bool ObstacleFound(uint8_t distance1, uint8_t distance2){
      //the ultrasonic sensor only returns 0 if it cant detect anything(out of range). We use this to determine the 
    //                                                        AD added this here just in case for testing the ultrasonic sensort(testing)
     if((distance2 > 10 && distance2 < 60))/*||(distance1 <= 200 && (distance2 > 30 && distance2 < 70)) */
     {
       return true;
     }
    else
     {
       return false;      
     } 
}




//GYROSCOPE -------------------------------------------------------------------------------------------------------------------------------------------------------/ 
short Gyroscope(){
 //read the Gyroscope sensor
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

    //create custom mapping for the gyroscope to be used alongside with the pwm//main motor
    // ay and az are not used but I cannot comment them out because the program runs like 80% slower 
    
  short ax=-(Buf[0]<<8 | Buf[1]);
  short  ay=-(Buf[2]<<8 | Buf[3]);
  short az=Buf[4]<<8 | Buf[5];   
  

  short custom_ay = map(ax, -3800, 3800, 7, 53);
   return custom_ay;

}





//MOTOR FUNCTIONS---------------------------------------------------------------------------------------------------------------------------------------------------/
//MAIN FUNCTION 
void motorGo( uint8_t direction, uint8_t speed){         //Function that controls the variables: motor(0 ou 1), directionion (1 ou 2) e pwm (entra 0 e 255);{
  
  
    if(direction == 1)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direction == 2)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, speed); 
  
}


//AVOID OBSTACLE FUNCTION 
void AvoidObstacle(){
 //(testing)
 //while(1)  Park();  
     digitalWrite(EN_PIN_1, HIGH);
     digitalWrite(EN_PIN_2, HIGH); 
     myservo.write(90); 
  //motorGo(2, 60); 
  //delay(1000); 
   
  myservo.write(120); 
  delay(50); 
  motorGo(1, 63); 
  delay(600);
  motorGo(0,0); 
   
  myservo.write(60);
    delay(50);  
  motorGo(1, 60); 
  delay(550); 

  myservo.write(120); 
    delay(50); 
  motorGo(1, 60); 
  delay(150); 

  /*myservo.write(120); 
  motorGo(1, 10); 
  delay(3000);

  myservo.write(60);  
  motorGo(1, 10); 
  delay(3000);
  */
 // myservo.write(90); 
   
  
}
//PARK FUNCTION
void Park(){
  motorGo(2, 90);
  delay(50); 
  while(1){
    //(testitng)
  Serial.println("PARKED");   
  motorGo(0, 0);  
}
}






//SETUP AND LOOP FUNCTIONS ---------------------------------------------------------------------------------------------------------------------------------------------------/ 
void setup() {
  Wire.begin(); 
  Serial.begin(31250);

  //SERVO  
  myservo.attach(9);
  //SET TO DEFAULT IN CASE ITS NOT STRAIGHT 
    myservo.write(90); 


  //IR sensors
  pinMode(SensorPinA, INPUT);
  pinMode(SensorPinB, INPUT);
  pinMode(SensorPinC, INPUT);
  pinMode(SensorPinD, INPUT);
  pinMode(SensorPinE, INPUT);

  //MOTOR SENSORS
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  //ULTRASONIC SENSOR
  pinMode(trigPin, OUTPUT);
  pinMode(11, INPUT); 
  pinMode(12, INPUT); 
  pinMode(13, OUTPUT); 
  //ENABLE PINS FOR THE MOTOR
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  //GYROSCOPE SENSOR 

  //Set accelerometers low pass filter at 5Hz
  I2CwriteByte(0x68,29,0x06);
  
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(0x68,26,0x06);

  // Configure gyroscope range
  I2CwriteByte(0x68,27,0x18);

  // Configure accelerometers range
  I2CwriteByte(0x68,28,0x18);
}

void loop() {
  //DEFINING ALL OUR VARIABLES 
  unsigned short ultra_2 = Ultrasonic(12); 
  unsigned short ultra_1 = Ultrasonic(11); 

  //print   value (testing)
   Serial.print("ULTRA_TOP: "); 
   Serial.print(ultra_1); 
   Serial.print("\t ULTRA_BOTTOM: "); 
   Serial.print(ultra_2); 
   short gyro = Gyroscope(); 

   


   
   /*if(ObstacleFound(ultra_1, ultra_2)){
     //print when the obstacle is being avoided (testing)
     AvoidObstacle();
     //motorGo(0, 0); 
     Serial.print("\n Parked\n Parked\n "); 
     Serial.print("\n AVOIDING OBSTACLE ");   

    // Park();  
   }else{
     */
  
         
      short Leftmost = digitalRead(SensorPinE);
      short Left = digitalRead(SensorPinD);
      short Centre = digitalRead(SensorPinC);
      short Right = digitalRead(SensorPinB);
      short Rightmost = digitalRead(SensorPinA);
      
      //print IR sensor value(testing)
      Serial.print("LL: ");
      Serial.print(Leftmost); 
      Serial.print(" L: ");
      Serial.print(Left); 
      Serial.print(" C: ");
      Serial.print(Centre); 
      Serial.print(" R: ");
      Serial.print(Right); 
      Serial.print(" RR: ");
      Serial.print(Rightmost);        

      short numberOfSensorsOn = Leftmost + Left + Centre + Right + Rightmost;

      motorGo(1, 67);
        if(numberOfSensorsOn == 5 || (Leftmost == 1 && Rightmost == 1 && Centre == 1)){
        
          Park();            
        }else{
            
            short speed = gyro;// * MULTI;
            
            //pring gyroscope value (testing)            
            Serial.print("\t GYROSCOPE: " );
            Serial.print(gyro);  
            

            
            Steer(numberOfSensorsOn,  Leftmost,  Left,  Centre,  Right,  Rightmost);
                  
                     if(gyro <= 28 ){

                          motorGo(2, 20); 
                          delay(500); 
                          Serial.print("Break!!");
                      }    
                      else if(gyro >= 33){
                          Serial.print("kurec "); 
                           motorGo(1, 230); 
                           delay(800); 
                           Serial.print("\t SPEED: " ); 
                           Serial.print(speed+110);
                      }                    
                      else{                  
                        speed +=15;     
                         motorGo(1, speed ); 
                          Serial.print("\t SPEED: " ); 
                           Serial.print(speed);
                      }

                      
                      //print direction (testing)                      
                      //Serial.print("\t Straight: "); 
                      //Serial.print(straight); 
                      
                       //Enable pins for the motor
                   digitalWrite(EN_PIN_1, HIGH);
                   digitalWrite(EN_PIN_2, HIGH);
  
            
        }
   Serial.println(); 

   }
//}





