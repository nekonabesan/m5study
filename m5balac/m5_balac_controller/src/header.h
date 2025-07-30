#include <M5StickC.h>
#include <Wire.h>
#include <MPU6050.h>

void caliblation1(void);
void caliblation2(void);
void checkButtonP(void);
void calDelay(int n);
void setMode(bool inc);
void startDemo(void);
void resetPara(void);
void getGyro(void);
void readGyro(void);
void driver(void);
void driveMotorL(int16_t power);
void driveMotorR(int16_t power);
void driveMotor(byte ch, int8_t speed);
void resetMotor(void);
void resetVar(void);
void sendStatus(void);
void imuInit(void);
void displayBatteryVoltage(void);
