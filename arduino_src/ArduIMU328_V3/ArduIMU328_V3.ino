/*
Authors:        Jordi Munoz, Jose Julio, Pat Hickey
Thanks to:      Chris Anderson


This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.


*/

#include <SPI.h>
#include <Wire.h>

#define MPU6000_CHIP_SELECT_PIN 4

// MPU 6000 registers

#define MPUREG_WHOAMI 0x75 //
#define	MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define	MPUREG_INT_ENABLE 0x38 
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 // 
#define	MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define	MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define	MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define	MPUREG_PWR_MGMT_1 0x6B //
#define	MPUREG_PWR_MGMT_2 0x6C //

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define	BIT_INT_ANYRD_2CLEAR	    0x10
#define	BIT_RAW_RDY_EN		    0x01
#define	BIT_I2C_IF_DIS              0x10

int SENSOR_SIGN[9] = {1,-1,-1,1,1,1,-1,-1,-1};  //{1,-1,-1,1,1,1,-1,-1,-1};  //Correct directions x,y,z - gyros, accels, magnetormeter

int MPU6000_newdata;

  int accelX;
  int accelY;
  int accelZ;
  unsigned int temp;
  int gyroX;
  int gyroY;
  int gyroZ;

  int magnetom_x;
  int magnetom_y;
  int magnetom_z;
  float MAG_Heading;



void setup()
{
  Serial.begin(115200);
  Serial.println("ArduPilot Me 3 Test");
  mpu_setup();    //MPU Initialization  
  Compass_Init();
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  delay(1000);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
}

void loop()
{
  
 mpu_read ();
 Read_Compass();
 Compass_Heading();
 
   Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print ("\t");
  
  Serial.print("Accels: ");
  Serial.print(accelX/255);
  Serial.print ("\t");
  Serial.print(accelY/255);
  Serial.print ("\t");
  Serial.print(accelZ/255);
  Serial.print ("\t");
  
  Serial.print("Gyros: ");
  Serial.print(gyroX/255);
  Serial.print ("\t");
  Serial.print(gyroY/255);
  Serial.print ("\t");
  Serial.print(gyroZ/255);
  Serial.print ("\t");
  
  Serial.print("Mag: ");
  Serial.print(magnetom_x);
  Serial.print ("\t");
  Serial.print(magnetom_y);
  Serial.print ("\t");
  Serial.print(magnetom_z);
  Serial.print ("\t");
  Serial.println();
 
 delay(50);
}
