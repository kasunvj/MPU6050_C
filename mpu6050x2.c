#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>

char mpu6050_1;
char mpu6050_2;
char multiGyro = 1;

//Sensor Reset
//IN: Device handler
//Return: None
//0x6B is the power register. setting MSB to 1(0x6B <= 0x80:1000 000), reset the sensor and 
//  come to a sleeping state. after resetting, MSB automatically set to 0
//  and sleeping seeting the 7th bit high(when this happen, the value of 
//  the 0x6B register is 0x40 = 64:0100 0000). You can wakeup the sensor 
//  by setting the 7th bit low. Or writing 0x6B register 0x00
void reset(char device){
    wiringPiI2CWriteReg8(device,0x6B,0x80);
    while(wiringPiI2CReadReg8(device,0x6B) != 0x40){}
    printf("Device reset done\n");
}

//Sensor Set
//IN: Device handler
//Return: None
void set(char device){
    wiringPiI2CWriteReg8(device,0x6B,0x00); //waking up  
}

void configSummary(char device){
    char range = wiringPiI2CReadReg8(device,0x1B);
    char r = 0;
    if(range == 0){r = 250;}
    else if(range == 8){r = 500;}
    else if(range == 16){r = 1000;}
    else if(range == 24){r = 2000;}
    printf("Range : +/- %d deg per sec\n",r);
}

int main(){
    mpu6050_1 = wiringPiI2CSetup (0x68);
    // The default value for the sensor is 0x68, that can be found in 
    //  reading 0x75 register(WHO_AM_I). the default value of that register is 0x68. 
    //  by applying 5V to AD0 pin in the second sensor address make the sensor 
    //  address to 0x69. but it is not refllected in WHO_AM_I register 
    if(multiGyro == 1){mpu6050_2 = wiringPiI2CSetup(0x69);}
    
    set(mpu6050_1);
    if(multiGyro == 1){set(mpu6050_2);};

    configSummary(mpu6050_1);

    printf("%x\n",wiringPiI2CReadReg8(mpu6050_1,0x6B));
    if(multiGyro == 1){printf("%x\n",wiringPiI2CReadReg8(mpu6050_2,0x6B));};
    return 0;
}