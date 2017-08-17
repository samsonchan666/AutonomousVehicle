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
enum { MAGNET, ACC_METER, GYRO };

struct Accelerometer{
	int16_t calibrated_x = 0 , calibrated_y = 0, calibrated_z = 0;
	int16_t raw_x, raw_y, raw_z;
};

struct Magnet
{
	int16_t calibrated_x, calibrated_y, calibrated_z;
	int16_t raw_x, raw_y, raw_z;
	float heading;
};

class IMU{
private:
	Accelerometer acc_struct;
	Magnet mag_struct;
	ADXL345 acc ;
	HMC5883L mag ;	
public:
	IMU() {
	  printf("ADXL345 3-axis acceleromter example program\n");
	  printf("HMC5883L 3-axis magnet example program\n");
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

	  acc.initialize();

	  mag.initialize();
	  mag.setSampleAveraging(HMC5883L_AVERAGING_8);
	  mag.setGain(HMC5883L_GAIN_1090);
	
	}
	~IMU() {}

	Accelerometer getAccelerometer(){ return acc_struct;}
	Magnet getMagnet() { return mag_struct;}

	void run_sensors(){
	    acc.getAcceleration(&acc_struct.raw_x, &acc_struct.raw_y, &acc_struct.raw_z);
	    compensate_sensor_errors(acc_struct.raw_x, acc_struct.raw_y, acc_struct.raw_z, ACC_METER);
	    mag.getHeading(&mag_struct.raw_x, &mag_struct.raw_y, &mag_struct.raw_z);
	    compensate_sensor_errors(mag_struct.raw_x, mag_struct.raw_y, mag_struct.raw_z, MAGNET);
	    mag_struct.heading = atan2(mag_struct.calibrated_y, mag_struct.calibrated_x)  * 180 / PI;

	    printf("  ax:  %5d       ay:  %5d      az:  %5d,       mx:  %3d       my:  %3d      mz:  %3d     heading:  %3.1f deg\n"
	      , acc_struct.calibrated_x, acc_struct.calibrated_y, acc_struct.calibrated_z, 
	      mag_struct.calibrated_x, mag_struct.calibrated_y, mag_struct.calibrated_z, mag_struct.heading); 

	    // write to logfile
	    // dataLog << "ax:  " << ax << "      ay:  " << ay << "     az:  " << az;
	    // dataLog << ",       mx:  " << mx << "       my:  " << my << "       mz:  " << mz << "     heading:  " << heading <<  " deg" << endl;

	    fflush(stdout);
	    bcm2835_delay(200);	
	}

	void compensate_sensor_errors(int16_t  x, int16_t  y, int16_t  z, int SENSOR) {
	  switch(SENSOR){
	    case MAGNET: {
	      float x_max = 450; float x_min = -421; float y_max = 325; float y_min = -549;
	      float z_max = 555; float z_min = -549;
	      float x_offset = (x_max + x_min) / 2.0; 
	      float y_offset = (y_max + y_min) / 2.0;  
	      float z_offset = (z_max + z_min) / 2.0;
	      float x_scale = 100.0/(x_max - x_offset); float y_scale = 100.0/(y_max - y_offset);
	      float z_scale = 100.0/(z_max - z_offset);

	      mag_struct.calibrated_x = (float(x) - x_offset) * x_scale;
	      mag_struct.calibrated_y = (float(y) - y_offset) * y_scale;
	      mag_struct.calibrated_z = (float(z) - z_offset) * z_scale;
	      break;
	    }
	    case ACC_METER: {
	      float gravity = 256.0f; // "1G reference" used for DCM filter and accelerometer calibration
	      float x_max = 261; float x_min = -254; float y_max = 266; float y_min = -248;
	      float z_max = 250; float z_min = -250;
	      float x_offset = (x_max + x_min) / 2.0; 
	      float y_offset = (y_max + y_min) / 2.0;  
	      float x_cal_offset = 12.5; float y_cal_offset = -7.5;
	      
	      float z_offset = (z_max + z_min) / 2.0;   
	      float x_scale = gravity/(x_max - x_offset); float y_scale = gravity/(y_max - y_offset);
	      float z_scale = gravity/(z_max - z_offset);

	      acc_struct.calibrated_x = ((float(x) - x_offset) * x_scale) - x_cal_offset;
	      acc_struct.calibrated_y = ((float(y) - y_offset) * y_scale) - y_cal_offset;
	      acc_struct.calibrated_z = (float(z) - z_offset) * z_scale;
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