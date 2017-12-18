
#ifndef __LSM303D_H
#define __LSM303D_H
#include "mbed.h"



class LSM303D {
    public:
        // Full-scale paramated -+2g,-+4g,-+6g,-+10g,-+16g,
        typedef enum{
            ACC_2G  =   0x00,
            ACC_4G  =   0x08,
            ACC_6G  =   0x10,
            ACC_8G  =   0x18,
            ACC_16G =   0x20,
        }ACC_PREC_E;

        /** Create a new interface for an LSM303D
         *
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        LSM303D(PinName sda, PinName scl,ACC_PREC_E prec);

  
        /** read the raw accelerometer and compass values
         *
         * @param ax,ay,az is the accelerometer 3d vector, written by the function
         * @param mx,my,mz is the magnetometer 3d vector, written by the function
         */
         bool read(float *ax, float *ay, float *az, float *mx, float *my, float *mz);
         bool read_acce(float *ax, float *ay, float *az);


    private:
        I2C _LSM303;

        ACC_PREC_E    m_acc_prec;
        float       m_sensi;//Linear acceleration sensitivity    
         
        bool write_reg(int addr_i2c,int addr_reg, char v);
        bool read_reg(int addr_i2c,int addr_reg, char *v);
        bool recv(char sad, char sub, char *buf, int length);
};

#endif