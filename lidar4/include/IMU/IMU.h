#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <bcm2835.h>
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "ADXL345.h"
#include "ITG3200.h"

#define PI 3.14159265
//#define NO_OF_SAMPLE 100

enum { MAGNET, ACC_METER, GYRO };

struct Accelerometer{
	int16_t ax;
	int16_t ay;
	int16_t az;
};

struct Magnet
{
	int16_t mx, my, mz;
	float heading;
};

struct Gyro
{
	int16_t gx, gy, gz;
};

class IMU{
private:       
	Accelerometer acc_struct;
	Magnet mag_struct;
        Gyro gyro_struct;
	ADXL345 acc ;
	HMC5883L mag ;
        ITG3200 gyro;
        
public:
	IMU() {
	  printf("ADXL345 3-axis acceleromter example program\n");
	  printf("HMC5883L 3-axis magnet example program\n");
          printf("ITG3200 3-axis gyroscope example program\n");
	  I2Cdev::initialize();
	  bcm2835_delay(2000);

	  //bcm2835_delay(200);

	  // Accelerometer connection test
	  if ( acc.testConnection() ) {
	      bcm2835_delay(2000);
	    printf("ADXL345 connection test successful\n") ;
	  }
	  else {
	    fprintf( stderr, "ADXL345 connection test failed! something maybe wrong, continueing anyway though ...\n");
	    //return 1;
	  }

	  // Magnet connection test
	  if ( mag.testConnection() ) {
	      bcm2835_delay(2000);
	    printf("HMC5883L connection test successful\n") ;
	  }
	  else {
	    fprintf( stderr, "HMC5883L connection test failed! something maybe wrong, continueing anyway though ...\n");
	    //return 1;
	  }
          
          //Gyro connection test
          if ( gyro.testConnection() ) {
            bcm2835_delay(2000);
          printf("ITG3200 connection test successful\n") ;
          }
           else {
            fprintf( stderr, "ITG3200 connection test failed! something maybe wrong, continueing anyway though ...\n");
            //return 1;
          }
          
	  acc.initialize();

	  mag.initialize();
	  mag.setSampleAveraging(HMC5883L_AVERAGING_8);
	  mag.setGain(HMC5883L_GAIN_1090);
          
          gyro.initialize();
	
	}
	~IMU() {}

	Accelerometer getAccelerometer(){ return acc_struct;}
	Magnet getMagnet() { return mag_struct;}

	bool run_sensors(){
	    acc.getAcceleration(&acc_struct.ax, &acc_struct.ay, &acc_struct.az);
	    compensate_sensor_errors(&acc_struct.ax, &acc_struct.ay, &acc_struct.az, ACC_METER);
//            calAvrAcc();
            
            if (acc_struct.ax > 256 || acc_struct.ax < -256) return false;  //Extreme value
            else if (acc_struct.ay > 256 || acc_struct.ay < -256) return false;
            //else if (acc_struct.az > 256 || acc_struct.az < -256) return false; //Don't care about z right now
            
	    mag.getHeading(&mag_struct.mx, &mag_struct.my, &mag_struct.mz);
	    compensate_sensor_errors(&mag_struct.mx, &mag_struct.my, &mag_struct.mz, MAGNET);
	    mag_struct.heading = atan2(mag_struct.my, mag_struct.mx)  * 180 / PI;
            
            gyro.getRotation(&gyro_struct.gx, &gyro_struct.gy, &gyro_struct.gz);            

//	    printf("  ax:  %5d       ay:  %5d      az:  %5d,       mx:  %3d       my:  %3d      mz:  %3d     heading:  %3.1f deg\n"
//	      , acc_struct.ax, acc_struct.ay, acc_struct.az, 
//	      mag_struct.mx, mag_struct.my, mag_struct.mz, mag_struct.heading); 
            
	    // write to logfile
	    // dataLog << "ax:  " << ax << "      ay:  " << ay << "     az:  " << az;
	    // dataLog << ",       mx:  " << mx << "       my:  " << my << "       mz:  " << mz << "     heading:  " << heading <<  " deg" << endl;

            //printf("  gx:  %d       gy:  %d      gz:  %d     \n", gx, gy, gz);  
                                  
	    fflush(stdout);
	    //bcm2835_delay(200);
            return true;
	}

	//Compensate wrongly scaled sensor axes, zero offsets
	void compensate_sensor_errors(int16_t*  x, int16_t*  y, int16_t*  z, int SENSOR) {
	  switch(SENSOR){
	    case MAGNET: {
	      float x_max = 450; float x_min = -421; float y_max = 325; float y_min = -549;
	      float z_max = 555; float z_min = -549;
	      float x_offset = (x_max + x_min) / 2.0; 
	      float y_offset = (y_max + y_min) / 2.0;  
	      float z_offset = (z_max + z_min) / 2.0;
	      float x_scale = 100.0/(x_max - x_offset); float y_scale = 100.0/(y_max - y_offset);
	      float z_scale = 100.0/(z_max - z_offset);

              (*x) = (float(*x) - x_offset) * x_scale;
	      (*y) = (float(*y) - y_offset) * y_scale;
	      (*z) = (float(*z) - z_offset) * z_scale;
	      break;
	    }
	    case ACC_METER: {
	      float gravity = 256.0f; // "1G reference" used for DCM filter and accelerometer calibration
	      float x_max = 261; float x_min = -254; float y_max = 266; float y_min = -248;
	      float z_max = 250; float z_min = -250;
	      float x_offset = (x_max + x_min) / 2.0; 
	      float y_offset = (y_max + y_min) / 2.0;  
	      float x_cal_offset = 16.559-2.738+3.275; float y_cal_offset = -81.583-0.699+12.137+1.5; 
	      
	      float z_offset = (z_max + z_min) / 2.0;   
	      float x_scale = gravity/(x_max - x_offset); float y_scale = gravity/(y_max - y_offset);
	      float z_scale = gravity/(z_max - z_offset);

	      (*x) = ((float(*x) - x_offset) * x_scale) - x_cal_offset;
	      (*y) = ((float(*y) - y_offset) * y_scale) - y_cal_offset;
	      (*z) = (float(*z) - z_offset) * z_scale;
	      break;
	    }
	    case GYRO: {
	      break;
	    }
	    default: break;
  		}

	}


};

#endif