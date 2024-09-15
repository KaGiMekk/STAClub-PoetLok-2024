#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

unsigned long Present_time , Previous_time;
float Delta_time ;
int Present_state ,L_Next_state,R_Next_state;
float Present_error,Intergral_error,Devirative_error,Previous_error ;
int prev =0;


// Tuner
float offset = 60;
float set_point = 0 ;


Servo FIN_Left;
Servo FIN_Right;
MPU6050 mpu(Wire);
long timer = 0;

float PID (float input ,float Min_out , float Max_out, float kp , float ki , float kd )
{
  Present_time = micros();
  Delta_time = (float)(Present_time  - Previous_time);
  
  Present_error   = set_point - input ;
  Intergral_error  += Present_error*Delta_time;
  Devirative_error  = (Present_error - Previous_error)/Delta_time ;
  
  if (Intergral_error >= Max_out ) Intergral_error = Max_out;
  else if(Intergral_error <= Min_out )Intergral_error = Min_out;
  
  float out = (kp*Present_error) + (ki*Intergral_error) + (kd*Devirative_error) ;

  if (out >= Max_out ) out = Max_out;
  else if (out <= Min_out ) out = Min_out;
  
  Previous_error = Present_error;
  Previous_time  = Present_time;
  return out;
}





void setup() 
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  FIN_Left.attach(19);
  FIN_Right.attach(18);
  
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  FIN_Left.write(offset);  
  FIN_Right.write(offset); 
}

void loop() 
{


// Test MPU
/*
  mpu.update();
  if(millis() - timer > 100){ // print data every second
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }
*/



// Test Servo
/*
    FIN_Left.write(60);  
    FIN_Right.write(60);         
    delay(1000);
    FIN_Left.write(0);  
    FIN_Right.write(0);        
    delay(1000);                     
    FIN_Left.write(60);  
    FIN_Right.write(60);           
    delay(1000);
    FIN_Left.write(180);  
    FIN_Right.write(180);           
    delay(1000);
*/

  digitalWrite(LED_BUILTIN, HIGH); 
  mpu.update();
  if(millis() - timer > 10)
  { 
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    
    Present_state = mpu.getAngleZ();
    
    L_Next_state = PID(Present_state ,-60,120,0.8,0.5,0.6);
    R_Next_state = PID(Present_state ,-120,60,0.7,0.4,0.5);
    
    FIN_Left.write(offset+L_Next_state);  
    FIN_Right.write(offset-R_Next_state); 
    
    Serial.println(F("=====================================================\n"));
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.print("\tNext_state: ");Serial.println(L_Next_state);
    Serial.print("\tFIN_Left: ");Serial.println(offset+L_Next_state);
    Serial.print("\tFIN_Right: ");Serial.println(offset-R_Next_state);
    timer = millis();
  }
  

  


}
