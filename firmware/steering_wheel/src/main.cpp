#include<Wire.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#define USE_USBCON

const int A = 8;//L1
const int B = 9;//L2
const int C = 10;//R1
const int D = 11;//R2

//Endereco I2C do MPU6050
const int MPU=0x68;

int8_t ACCEL_XOUT_H;
int8_t ACCEL_XOUT_L;
int8_t ACCEL_YOUT_H;
int8_t ACCEL_YOUT_L;
uint8_t B_CTRL;

ros::NodeHandle  nh;	
std_msgs::Int8 power_ref, power_dif;
std_msgs::UInt8  ctrl_mode;

ros::Publisher pub_power_ref("/power_ref", &power_ref);
ros::Publisher pub_power_dif("/power_dif", &power_dif);
ros::Publisher pub_ctrl_mode("/ctrl_mode", &ctrl_mode);
//---------------------------------------
void setup()
{
    pinMode(A, INPUT);
    pinMode(B, INPUT);
    pinMode(C, INPUT);
    pinMode(D, INPUT);

    nh.initNode();
    nh.advertise(pub_power_ref); 
    nh.advertise(pub_power_dif);
    nh.advertise(pub_ctrl_mode);

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
    
    B_CTRL = 0x00;
    if(digitalRead(A)==LOW){
        B_CTRL = 0b00000001;
    }
    if(digitalRead(B)==LOW){
        B_CTRL |= 0b00000010;
    }
    if(digitalRead(C)==LOW){
        B_CTRL |= 0b00000100;
    }   
    if(digitalRead(D)==LOW){
        B_CTRL |= 0b00001000;
    }

    ctrl_mode.data = B_CTRL;
    pub_power_ref.publish(&power_ref);
    pub_power_dif.publish(&power_dif);
    pub_ctrl_mode.publish(&ctrl_mode); 

    //---------------------------------------
    nh.spinOnce();
    delay(10); 	
}