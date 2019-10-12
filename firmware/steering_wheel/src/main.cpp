#include<Wire.h>
#include <ros.h>
#include <std_msgs/Int8.h>

#define USE_USBCON

//Endereco I2C do MPU6050
const int MPU=0x68;

int8_t ACCEL_XOUT_H;
int8_t ACCEL_XOUT_L;
int8_t ACCEL_YOUT_H;
int8_t ACCEL_YOUT_L;

ros::NodeHandle  nh;	
std_msgs::Int8 power_ref, power_dif;

ros::Publisher pub_power_ref("power_ref", &power_ref);
ros::Publisher pub_power_dif("power_dif", &power_dif);

//---------------------------------------
void setup()
{
    nh.initNode();
    nh.advertise(pub_power_ref); 
    nh.advertise(pub_power_dif);

    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);

    //Inicializa o MPU-6050
    Wire.write(0); 
    Wire.endTransmission(true); 
}

void loop()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    
    //Solicita os dados do sensor
    Wire.requestFrom(MPU, 14, true); 

    //Armazena o valor dos sensores nas variaveis correspondentes
    ACCEL_XOUT_H = Wire.read(); //0x3B (ACCEL_XOUT_H)
    ACCEL_XOUT_L = Wire.read(); //0x3C (ACCEL_XOUT_L)     
    ACCEL_YOUT_H = Wire.read(); //0x3D (ACCEL_YOUT_H)
    ACCEL_YOUT_L = Wire.read(); //0x3E (ACCEL_YOUT_L)

    power_ref.data = ACCEL_XOUT_H;
    power_dif.data = ACCEL_YOUT_H;

    pub_power_ref.publish(&power_ref);
    pub_power_dif.publish(&power_dif);

    //---------------------------------------
    nh.spinOnce();
    delay(300); 	
}