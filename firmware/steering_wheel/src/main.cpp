#include<Wire.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#define USE_USBCON

const int L1 = 8;
const int L2 = 9;
const int R1 = 10;
const int R2 = 11;

//Endereco I2C do MPU6050
const int MPU=0x68;

int8_t ACCEL_XOUT_H;
int8_t ACCEL_XOUT_L;
int8_t ACCEL_YOUT_H;
int8_t ACCEL_YOUT_L;
int8_t B_CTRL;

ros::NodeHandle  nh;	
std_msgs::Int8 power_ref, power_dif, ctrl;

ros::Publisher pub_power_ref("power_ref", &power_ref);
ros::Publisher pub_power_dif("power_dif", &power_dif);
ros::Publisher pub_ctrl("ctrl", &ctrl);
//---------------------------------------
void setup()
{
    pinMode(L1, INPUT);
    pinMode(L2, INPUT);
    pinMode(R1, INPUT);
    pinMode(R2, INPUT);

    nh.initNode();
    nh.advertise(pub_power_ref); 
    nh.advertise(pub_power_dif);
    nh.advertise(pub_ctrl);

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
    if(digitalRead(L1)==LOW){
        B_CTRL = 0b00000001;
        pub_power_ref.publish(&power_ref);
    }
    if(digitalRead(R1)==LOW){
        B_CTRL |= 0b00000010;
        pub_power_dif.publish(&power_dif);
    }   
    if(digitalRead(L2)==LOW){
        B_CTRL |= 0b00000100;
        power_dif.data = -125;
        pub_power_dif.publish(&power_dif);
    }
    if(digitalRead(R2)==LOW){
        B_CTRL |= 0b00001000;
        power_dif.data = 125;
        pub_power_dif.publish(&power_dif);
    }
    ctrl.data = B_CTRL;
    pub_ctrl.publish(&ctrl); 

    //---------------------------------------
    nh.spinOnce();
    delay(10); 	
}