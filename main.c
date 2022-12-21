//i2cdetect -y 1
// ./main data.csv
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <softTone.h>

//Motor/encoder
#define PWM0 13
#define PWM1 19
#define BUZZ 12
#define ENCA 24
#define ENCB 23
#define GYROADD1 0x68 // 0x68: ;ocated closer to the arm 0x69 locates far from arm
#define GYROADD2 0x69
#define XREG 0x43
#define YREG 0x45
#define ZREG 0x47



//Gyro
struct gyro{
    char mpu6050;
    int mesurement_range;
    double gyroX, gyroY, gyroZ;
    double offsetX,offsetY,offsetZ;
}gyro1,gyro2;



int count = 0;
int current_count;
int previous_count;
double degs_per_count = 4.5;
double checkGyro;
double checkGyro2;
double thetaDotError; //comming from gyro
double thetaDotMeasure; //comming from encoder
double thetaError;
int scale [8] = { 262, 294, 330, 349, 392, 440, 494, 525 } ;
double spotCallibrationOffset ;
double spotCallibrationOffset2 ;


void set();
void counterA(void);
// void calThetaDot();
void timer_callback(int signam);
int readword(char device, char addr);
double sample_average(char device, char reg);
double get(char device, char reg);
void rotate(double ref,char *file);
double get2(char device, char reg);
double sample_average2(char device,char reg);
 

int main(int argc, char *argv[]){
    char *fname = argv[1];
    FILE *fpt;
    
	
	
    
    wiringPiSetupGpio();
    pinMode(PWM0, OUTPUT);
    pinMode(PWM1, OUTPUT);
    softToneCreate(BUZZ);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pullUpDnControl(ENCA,PUD_DOWN);
    pullUpDnControl(ENCB,PUD_DOWN);

    if (wiringPiISR(ENCA, INT_EDGE_RISING,counterA) <0 ){
        printf("ISR setting up error\n");
        return -1; 
        }

    gyro1.mpu6050 = wiringPiI2CSetup (GYROADD1); //I2C setup
    gyro2.mpu6050 = wiringPiI2CSetup (GYROADD2); //I2C setup
    set();

    digitalWrite(PWM0, LOW);
    digitalWrite(PWM1, LOW);

    softPwmCreate(PWM0,0,100);
    softPwmCreate(PWM1,0,100);

    signal(SIGALRM,timer_callback);
    alarm(1);


    for (int i = 0; i<5 ;i++){
        printf("Ready .. %ds\n",5-i);
        softToneWrite (BUZZ, scale [1]) ;
        usleep(500000);
        softToneWrite (BUZZ, 0) ;
        usleep(500000);
    }
    for (int i = 0; i<3 ;i++){
        printf("Calibrating .. %ds\n",3-i);
        spotCallibrationOffset += get(gyro1.mpu6050,ZREG);
        spotCallibrationOffset2 += get(gyro2.mpu6050,ZREG);
        usleep(1000000);
    }

    for (int k = 0; k<5; k++){
        softToneWrite (BUZZ, scale [k]) ;
        usleep(50000);
        softToneWrite (BUZZ, 0) ;
        usleep(50000);
    }
    spotCallibrationOffset = spotCallibrationOffset/3;
    spotCallibrationOffset2 = spotCallibrationOffset2/3;
    printf("Spot Callibration offset 1: %f\n",spotCallibrationOffset);
    printf("Spot Callibration offset 2: %f\n",spotCallibrationOffset2);
    

    while(1){
        fpt = fopen(fname,"a");
        checkGyro = get(gyro1.mpu6050,ZREG) - spotCallibrationOffset;
        checkGyro2 = get(gyro2.mpu6050,ZREG) - spotCallibrationOffset2;

        //printf("Theta Ref %f\n",checkGyro);

        if(abs((int)checkGyro)>2){
            fprintf(fpt,"%f,%f,%f,%d,%d\n",checkGyro,checkGyro2,checkGyro-checkGyro2, 0, count);
            thetaDotError = checkGyro;
            rotate(thetaDotError,fpt) ;//-:CW:PWM1 , +:CCW:PWM0   
            
            }
        else fprintf(fpt,"%f,%f,%f,%d,%d\n",checkGyro,checkGyro2,checkGyro-checkGyro2, 0, count);
        
        usleep(100000);
        }
       
    
    digitalWrite(PWM0, LOW);
    digitalWrite(PWM1, LOW);
    
    return 0;
    }

void rotate(double ref,char *file){
    char PWMPIN ;
    int maxPWM;
    double K = 1;
    double KI = 0;
    double accuerror = 0;

    
    double error ;
    int thetaCountToRotate;
    

    error = ref ;
    thetaCountToRotate = (int)(K*(abs(error)/18)) ;// 18 degrees per count 

    while((abs(error) > 18)){

        count = 0;
        if(error < 0) PWMPIN = PWM0; 
        if(error > 0) PWMPIN = PWM1;

        while(count<thetaCountToRotate){
            softPwmWrite(PWMPIN, 100);
            checkGyro2 += get(gyro2.mpu6050,ZREG) - spotCallibrationOffset2;
            printf("Hand W: %f  Endeff W: %f Error: %f CalculatedCount: %d Currentcount: %d \n",ref,checkGyro2,error, thetaCountToRotate, count);
            usleep(1000);
        }

        if (thetaCountToRotate != 0) checkGyro2 = checkGyro2/thetaCountToRotate;
        else checkGyro2 = checkGyro2;

        
        fprintf(file,"%f,%f,%f,%d,%d\n",ref,checkGyro2,error, thetaCountToRotate, count);

        softPwmWrite(PWMPIN, 0);
        error = ref - 18*thetaCountToRotate ;
        accuerror = accuerror + error;
        //printf("Error After one action: %f\n",error);
        usleep(10000);
        thetaCountToRotate = (int)(((K*error+ KI*accuerror)/18));

    }

    fclose(file);
    
    

}


void set(){
    int range = wiringPiI2CReadReg8(gyro1.mpu6050,0x1B);//measurement range the current settings and allocate
    if(range == 0)       gyro1.mesurement_range = 250; 
    else if(range == 8)  gyro1.mesurement_range = 500; 
    else if(range == 16) gyro1.mesurement_range = 1000; 
    else if(range == 24) gyro1.mesurement_range = 2000;
    else                 gyro1.mesurement_range = 250;

    if(range == 0)       gyro2.mesurement_range = 250; 
    else if(range == 8)  gyro2.mesurement_range = 500; 
    else if(range == 16) gyro2.mesurement_range = 1000; 
    else if(range == 24) gyro2.mesurement_range = 2000;
    else                 gyro2.mesurement_range = 250;

    gyro1.offsetX = sample_average(gyro1.mpu6050,XREG);
    gyro1.offsetY = sample_average(gyro1.mpu6050,YREG);
    gyro1.offsetZ = sample_average(gyro1.mpu6050,ZREG);
    gyro2.offsetX = sample_average2(gyro2.mpu6050,XREG);
    gyro2.offsetY = sample_average2(gyro2.mpu6050,YREG);
    gyro2.offsetZ = sample_average2(gyro2.mpu6050,ZREG);

    wiringPiI2CWriteReg8(gyro1.mpu6050,0x6B,0x00); //waking up 
    wiringPiI2CWriteReg8(gyro2.mpu6050,0x6B,0x00); //waking up   
}
void counterA(void){
    count += 1;
    //printf("%d\n",count);
    
}
void timer_callback(int signam){
    current_count = count;
    //printf("%d %d %f\n",current_count,previous_count,thetaDotMeasure);
    thetaDotMeasure = (double)((current_count-previous_count)*degs_per_count);
    previous_count = current_count;
    alarm(1);
}

int readword(char device, char addr){
    int val = wiringPiI2CReadReg8(device,addr);

    val = val << 8;
    val += wiringPiI2CReadReg8(device, addr + 1);
    //full scale value(16bit) is 65536, which even represent negative values.
    //  0x8000 = 32728(= 65536/2)

    if (val >= 0x8000){
        val = -(65536 - val);
    }
    return val;
}
double get(char device, char reg){
    double offset;

    if(reg == XREG)      offset=gyro1.offsetX;
    else if(reg == YREG) offset=gyro1.offsetY;
    else if(reg == ZREG) offset=gyro1.offsetZ;
    
    return readword(device,reg)/(65536/gyro1.mesurement_range) - offset;
}
double get2(char device, char reg){
    double offset;

    if(reg == XREG)      offset=gyro2.offsetX;
    else if(reg == YREG) offset=gyro2.offsetY;
    else if(reg == ZREG) offset=gyro2.offsetZ;
    
    return readword(device,reg)/(65536/gyro2.mesurement_range) - offset;
}
double sample_average(char device,char reg){
    double data;

    for(int c=100 ; c>0 ;c--){
       data += readword(device,reg)/(65536/gyro1.mesurement_range);
    }
    return data/100; 
}

double sample_average2(char device,char reg){
    double data;

    for(int c=100 ; c>0 ;c--){
       data += readword(device,reg)/(65536/gyro2.mesurement_range);
    }
    return data/100; 
}


