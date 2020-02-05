#include <Arduino.h>

#include "MadgwickAHRS.h"
#include <Wire.h>
#include "IMU.h"
float quart[4];

int flag=true;

IMU::IMU(byte pin): pinNo(pin){
  pinMode(pinNo,OUTPUT);
  digitalWrite(pinNo,LOW);
  gxOffset=gyOffset=gzOffset =0;
  gyroRoll=gyroPitch=gyroYaw=0;
}

byte IMU:: PIN(){
  return pinNo;
}


void IMU::SETUP(){
  Wire.beginTransmission(deviceAd);
  Wire.write(pwrmgmt_ad);
  Wire.write(0b00000000);                 //Disable sleep mode                                         Put bit3 to 1 if you want to disable temperature sensor
  Wire.endTransmission();

  Wire.beginTransmission(deviceAd);
  Wire.write(gyro_ad);
  Wire.write(0b00000000);                         //Select degrees per second for gyro to read data accordingly   +-1000dps
  Wire.endTransmission();

  Wire.beginTransmission(deviceAd);
  Wire.write(accl_ad);
  Wire.write(0b00010000);                        ////Select range for accl to read data accordingly              +-8g
  Wire.endTransmission();
}

void IMU:: calibrateGyro(){
  Serial.println("Calibrating gyro...");
  delay(500);                               //Ignore data for half of a second
  for(int i=0;i<2000;i++){
     readData();
     gxOffset= gxOffset + gyX;
     gyOffset= gyOffset + gyY;
     gzOffset= gzOffset + gyZ;
     delay(5);
     if(i%200==0) Serial.println("...");
  }
  gxOffset= gxOffset/2000;
  gyOffset= gyOffset/2000;
  gzOffset= gzOffset/2000;
  Serial.println("Gyroscope Calibrated.");
  
}

void IMU::readData(){
  Wire.beginTransmission(deviceAd);      // Reading data from accl measurement registers
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(deviceAd, 6);
    accX= Wire.read()<<8 | Wire.read();
    accY= Wire.read()<<8 | Wire.read();
    accZ= Wire.read()<<8 | Wire.read();
     
  Wire.beginTransmission(deviceAd);     //Reading data from gyroscope measurement registers
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(deviceAd, 6);
    gyX= Wire.read()<<8 | Wire.read();
    gyY= Wire.read()<<8 | Wire.read();
    gyZ= Wire.read()<<8 | Wire.read();
     

  Wire.beginTransmission(deviceAd);     //  Reading data from temperature measurement registers
  Wire.write(0x41);
  Wire.endTransmission();
  Wire.requestFrom(deviceAd, 2);
  temperature= Wire.read()<<8 | Wire.read();
}



void IMU::calculateData(){
  
    Ax= accX/LSB_A;                 
    Ay= accY/LSB_A;
    Az= accZ/LSB_A;
    
  if(gxOffset>0)
       gyX=gyX-gxOffset;
    else
       gyX=gyX+gxOffset;
    
    if(gyOffset>0)
       gyY=gyY-gyOffset;
    else
       gyY=gyY+gyOffset;
    
    if(gzOffset>0)
       gyZ=gyZ-gzOffset;
    else
       gyZ=gyZ+gzOffset;


    Gx= gyX/LSB_G;        ///Complementary ko laagi         
    Gy= gyY/LSB_G;
    Gz= gyZ/LSB_G;

    /*Gx= gyX*radToDegree/LSB_G;        ///Quaternion ko laagi            
    Gy= gyY*radToDegree/LSB_G;
    Gz= gyZ*radToDegree/LSB_G;*/


 

 //  netAcc= sqrt(Ax*Ax +Ay*Ay +Az*Az);     //linear acceleration to provide proper direction (worse case scenario)
    T= temperature/340 + 36.53;            // Calculating temperature in Celsuis 
}



void IMU::calculateAngle(){             //DMP use garne yesmaa tara mildaina hola kina bhane 6 ota use garnu xa interrupts
 
 MadgwickAHRSupdateIMU(Gx,Gy,Gz,Ax,Ay,Az);   // event handler

  Serial.print("\nw:"); Serial.print(q0);
  Serial.print("  x:"); Serial.print(q1);
  Serial.print("  y:"); Serial.print(q2);
  Serial.print("  z:"); Serial.print(q3);
  
  roll= atan2(2.0*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
  pitch = asin(-2.0*(q1*q3 - q0*q2));
  yaw = atan2(2.0*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);

//To degrees again
  yaw*= 57.29;
  pitch*= 57.29;
  roll*= 57.29;
 
  

}

void IMU:: quaternion(){
  MadgwickAHRSupdateIMU(Gx,Gy,Gz,Ax,Ay,Az);

  roll= atan2(2.0*(q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);
  pitch = asin(-2.0*(q1*q3 - q0*q2));
  yaw = atan2(2.0*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);

//To degrees again
  yaw*= 57.29;
  pitch*= 57.29;
  roll*= 57.29;
 
  //pitch= acosf(q0/ sqrt(q0*q0 +q2*q2))*2.0f;      //utilze when data gets buggy
}

void IMU:: complementaryFilter(double sampleRate){
  
//Serial.print("\n\nSample Rate=");Serial.println(sampleRate);
  accRoll=  atan2(Ax,Az) * radToDegree;
  accPitch= atan2(Ay,Az) * radToDegree;

 /* if(flag==true){
  gyroRoll=accRoll;
  gyroPitch= accPitch;
  Serial.println("flag error");
  flag=false;
  }*/

  
  gyroRoll = gyroRoll + Gy* sampleRate;
  gyroPitch= gyroPitch + Gx * sampleRate;
  gyroYaw= gyroYaw + Gz * sampleRate;

  //tilt compensation jasto
 // pitch= pitch- roll * sin(gyroYaw* sampleRate* degToRadian);
 // roll= roll+ pitch  * sin(gyroYaw* sampleRate* degToRadian);

 roll  = 0.99 * gyroRoll +0.01 * accRoll;
 pitch = 0.99 * gyroPitch + 0.01 * accPitch;
}

void IMU::printData(){
   Serial.print("  Ax:");   Serial.print(Ax);      
   Serial.print("  Ay:");   Serial.print(Ay);
   Serial.print("  Az:");   Serial.print(Az);
   Serial.print("  Gx:");   Serial.print(Gx);     
   Serial.print("  Gy:");   Serial.print(Gy);     
   Serial.print("  Gz:");   Serial.print(Gz);   
   Serial.print("  T:");    Serial.println(T);

  Serial.print("  Roll:"); Serial.println(accRoll);
  Serial.print("  Pitch:"); Serial.println(accPitch);
  Serial.print("  Yaw:"); Serial.println(gyroYaw);
 }

void IMU:: toProcessing(){
 Serial.print(gyroYaw);
 Serial.print(","); 
 Serial.print(accPitch);  
 Serial.print(",");  
 Serial.print(accRoll);  
 Serial.print(",");
}

void IMU:: checkDevice(){
    byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {

    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

