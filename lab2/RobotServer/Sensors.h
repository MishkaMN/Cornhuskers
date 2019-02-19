#include <Wire.h>
#include <VL53L0X.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12
 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

void setupIMU();

void ReadIMU(float& ret_gz, float& ret_heading);

#define SDA_PORT 14
#define SCL_PORT 12

void ReadDistSensors(float& front, float& side, VL53L0X& sensor, VL53L0X& sensor2);
