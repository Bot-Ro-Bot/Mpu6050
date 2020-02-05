#ifndef IMU_h
#define IMU_h

#define device_adLeft 0x68         //AD0 pin connected to ground  **Note: 0x69 if AD0 is connected to VCC  (idea weak xa...kaam garne possibility 10%)
#define device_adRight 0x69
#define deviceAd 0x69
#define gyro_ad 0x1B
#define accl_ad 0x1C
#define pwrmgmt_ad 0x6B
#define LSB_A 4096.0
#define LSB_G 131.0
#define Pi 3.141592654
#define radToDegree (180/Pi)
#define degToRadian (Pi/180)


class IMU{
  private:
  
    const byte pinNo;
    int16_t accX,accY,accZ ,gyX,gyY,gyZ;
    int16_t temperature;
    float Gx,Gy,Gz,Ax,Ay,Az,netAcc;
    float T;
    float gxOffset,gyOffset,gzOffset;
    float gyroRoll,gyroPitch,gyroYaw;
    float accRoll,accPitch;
    float roll,pitch,yaw;
 public:
    IMU(byte pin);
    byte PIN();
    void SETUP();
    static void checkDevice();
    void calibrateGyro();
    void readData();
    void calculateData();
    void calculateAngle();
    void complementaryFilter(double sampleRate);
    void DMP();
    void quaternion();
    void toProcessing();
    void printData();
   
    
};


#endif
