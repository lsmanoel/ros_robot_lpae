#include <avr/io.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#define USE_USBCON

ros::NodeHandle  nh;
	
//---------------------------------------
#define ENA1 PB0
#define IN11 PB1
#define IN12 PB2
#define ENB1 PB3
#define IN13 PB4
#define IN14 PB5

#define ENA2 PC0
#define IN21 PC1
#define IN22 PC2
#define ENB2 PC3
#define IN23 PC4
#define IN24 PC5

#define IN31 PD0
#define IN32 PD1
#define IN33 PD2
#define IN34 PD3
#define IN35 PD4
#define IN36 PD5
#define IN37 PD6
#define IN38 PD7

//---------------------------------------
std_msgs::UInt8 dutycycle_L_msg, dutycycle_R_msg;
uint8_t dutycycle_L, dutycycle_R;
std_msgs::Bool spin_L_msg, spin_R_msg;
bool spin_L, spin_R;

//---------------------------------------
void dutycycle_L_callback(const std_msgs::UInt8 &dutycycle_L_msg)
{
    dutycycle_L = dutycycle_L_msg.data;
}
ros::Subscriber<std_msgs::UInt8> sub_power_L("/power_L", &dutycycle_L_callback);

//---------------------------------------
void spin_L_callback(const std_msgs::Bool &spin_L_msg)
{
    spin_L = spin_L_msg.data;

}
ros::Subscriber<std_msgs::Bool> sub_spin_L("/spin_L", &spin_L_callback);

//---------------------------------------
void dutycycle_R_callback(const std_msgs::UInt8 &dutycycle_R_msg)
{
    dutycycle_R = dutycycle_R_msg.data;
}
ros::Subscriber<std_msgs::UInt8> sub_power_R("/power_R", &dutycycle_R_callback);

//---------------------------------------
void spin_R_callback(const std_msgs::Bool &spin_R_msg)
{
    spin_R = spin_R_msg.data;

}
ros::Subscriber<std_msgs::Bool> sub_spin_R("/spin_R", &spin_R_callback);

//---------------------------------------
void setup()
{
    dutycycle_L = 0x00;
    dutycycle_R = 0x00;

    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD = 0xFF;

    PCMSK2 = 0x00;
    UCSR0B = 0x00;

    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
  
    nh.initNode();
    nh.subscribe(sub_power_L); 
    nh.subscribe(sub_spin_L);
    nh.subscribe(sub_power_R); 
    nh.subscribe(sub_spin_R);
}

void loop()
{
    PORTB = 0x00;
    PORTC = 0x00;
    for(uint8_t i=0; i<256; i++)
    {     
      //---------------------------------------
      if(i > dutycycle_L)
        PORTC = 0x00;
      else
        if(spin_R)
          PORTC = 0b00101101;
        else
          PORTC = 0b00011011;

      //---------------------------------------
      if(i > dutycycle_R)
        PORTB = 0x00;      
      else
        if(spin_L)
          PORTB = 0b00101101;
        else
          PORTB = 0b00011011;

      //---------------------------------------
      nh.spinOnce(); 
    }   	
}