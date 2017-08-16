/* 
 * File:   main.cpp
 * Author: osc
 *
 * Created on 7. august 2016, 15:06
 */
/*using http://wiringpi.com/reference/i2c-library/ */
#include <cstdlib>

using namespace std;
/*
I2Cdev library collection - HMC5883L RPi example
Based on the example in Arduino/HMC5883L/

==============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================

To compile on a Raspberry Pi (1 or 2)
  1. install the bcm2835 library, see http://www.airspayce.com/mikem/bcm2835/index.html
  2. enable i2c on your RPi , see https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
  3. connect your i2c devices
  4. then from bash
      $ PATH_I2CDEVLIB=~/i2cdevlib/
      $ gcc -o HMC5883L_example_1 ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/HMC5883L/examples/HMC5883L_example_1.cpp \
         -I ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp \
         -I ${PATH_I2CDEVLIB}/Arduino/HMC5883L/ ${PATH_I2CDEVLIB}/Arduino/HMC5883L/HMC5883L.cpp -l bcm2835 -l m
      $ sudo ./HMC5883L_example_1

*/

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

      *x = (float(*x) - x_offset) * x_scale;
      *y = (float(*y) - y_offset) * y_scale;
      *z = (float(*z) - z_offset) * z_scale;
      break;
    }
    case ACC_METER: {
      float gravity = 256.0f; // "1G reference" used for DCM filter and accelerometer calibration
      float x_max = 261; float x_min = -254; float y_max = 266; float y_min = -248;
      float z_max = 250; float z_min = -250;
      float x_offset = (x_max + x_min) / 2.0; 
      float y_offset = (y_max + y_min) / 2.0;  
      float z_offset = (z_max + z_min) / 2.0;   
      float x_scale = gravity/(x_max - x_offset); float y_scale = gravity/(y_max - y_offset);
      float z_scale = gravity/(z_max - z_offset);

      *x = (float(*x) - x_offset) * x_scale;
      *y = (float(*y) - y_offset) * y_scale;
      *z = (float(*z) - z_offset) * z_scale;
      break;
    }
    case GYRO: {
      break;
    }
    default: break;
  }

}

/*
//Acclerometer example
int main(int argc, char **argv) {
  printf("ADXL345 3-axis acceleromter example program\n");
  I2Cdev::initialize();
    bcm2835_delay(2000);
  ADXL345 acc ;
   bcm2835_delay(200);
  if ( acc.testConnection() ) {
      bcm2835_delay(2000);
    printf("ADXL345 connection test successful\n") ;
  }
  else {
    fprintf( stderr, "ADXL345 connection test failed! something maybe wrong, continueing anyway though ...\n");
    //return 1;
  }
  acc.initialize();

  int16_t ax, ay, az;
  while (true) {
    acc.getAcceleration(&ax, &ay, &az);
    printf("  ax:  %d       ay:  %d      az:  %d     \r", ax, ay, az);
    fflush(stdout);
    bcm2835_delay(200);
  }
  return 1; 
}
*/

/*
int main(int argc, char **argv) {
  printf("ITG3200 3-axis gyroscope example program\n");
  I2Cdev::initialize();
    bcm2835_delay(2000);
  ITG3200 gyro;
   bcm2835_delay(200);
  if ( gyro.testConnection() ) {
      bcm2835_delay(2000);
    printf("ITG3200 connection test successful\n") ;
  }
  else {
    fprintf( stderr, "ITG3200 connection test failed! something maybe wrong, continueing anyway though ...\n");
    //return 1;
  }
  gyro.initialize();
  int16_t gx, gy, gz;
  while (true) {
    gyro.getRotation(&gx, &gy, &gz);
    printf("  gx:  %d       gy:  %d      gz:  %d     \r", gx, gy, gz);
    fflush(stdout);
    bcm2835_delay(200);
  }
  return 1; 
}
*/


// Accelerometer and Magnet
int main(int argc, char **argv) {

  ofstream dataLog;
  dataLog.open("log.txt");

  printf("ADXL345 3-axis acceleromter example program\n");
  I2Cdev::initialize();
  bcm2835_delay(2000);
  ADXL345 acc ;
  HMC5883L mag ;
  bcm2835_delay(200);

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

  int16_t ax, ay, az;
  int16_t mx, my, mz;
  float heading ;  


  while (true) {
    acc.getAcceleration(&ax, &ay, &az);
    compensate_sensor_errors(&ax, &ay, &az, ACC_METER);
    mag.getHeading(&mx, &my, &mz);
    compensate_sensor_errors(&mx, &my, &mz, MAGNET);
    heading = atan2(my, mx)  * 180 / PI;

    printf("  ax:  %5d       ay:  %5d      az:  %5d,       mx:  %3d       my:  %3d      mz:  %3d     heading:  %3.1f deg\r"
      , ax, ay, az, mx, my, mz, heading); 

    // write to logfile
    dataLog << "ax:  " << ax << "      ay:  " << ay << "     az:  " << az;
    dataLog << ",       mx:  " << mx << "       my:  " << my << "       mz:  " << mz << "     heading:  " << heading <<  " deg" << endl;

    fflush(stdout);
    bcm2835_delay(200);
  }
  dataLog.close();
  return 1; 
}


/*
//Magnet example
int main(int argc, char **argv) {
  printf("HMC5883L 3-axis acceleromter example program\n");
  I2Cdev::initialize();
    bcm2835_delay(2000);
  HMC5883L mag ;
   bcm2835_delay(200);
  if ( mag.testConnection() ) {
      bcm2835_delay(2000);
    printf("HMC5883L connection test successful\n") ;
  }
  else {
    fprintf( stderr, "HMC5883L connection test failed! something maybe wrong, continueing anyway though ...\n");
    //return 1;
  }
  mag.initialize();
  mag.setSampleAveraging(HMC5883L_AVERAGING_8);
  mag.setGain(HMC5883L_GAIN_1090);
  int16_t mx, my, mz;
  float heading ;
  while (true) {
    mag.getHeading(&mx, &my, &mz);
    heading = atan2(my, mx)  * 180 / PI;
    printf("  mx:  %d       my:  %d      mz:  %d     heading:  %3.1f deg\r", mx, my, mz, heading);
    fflush(stdout);
    bcm2835_delay(200);
  }
  return 1; 
}
*/


/*
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>    // Used for printf() statements
#include <wiringPi.h> // Include WiringPi library!


	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};
int addr = 0x1E;          //<<<<<The I2C address of the slave
	char open(){
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 0;
	}
	return file_i2c;
	}
	char getBus(int addr ){
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
	}return -1;
	return 0;
	}
	
	void read(){
	//----- READ BYTES -----
	length = 4;			//<<< Number of bytes to read
	if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Data read: %s\n", buffer);
                printf("%i",length);
                for (int i=0; i<4; i++)
                printf(" " "%i",buffer[i]);
	}
	}
	void write(){
	//----- WRITE BYTES -----
	buffer[0] = 0x01;
	buffer[1] = 0x02;
	length = 2;			//<<< Number of bytes to write
	if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
	//	printf("Failed to write to the i2c bus.\n");
	//}
	//} 
/*       
int main(int argc, char** argv) {
open();
getBus(0x1E );
read();
	return 0;
}
*/

        /******************************************************************************************
* Test Program: Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/
/**
#include <iostream>   // cout()
#include <iomanip>    // setprecision() etc.
#include <stdexcept>  // runtime_error
#include <cstdio>     // getchar()
#include "RazorAHRS.h"

using namespace std;


// Set your serial port here!
//const string serial_port_name = "/dev/tty.FireFly-6162-SPP"; 
const string serial_port_name = "/dev/ttyAMA0";
//const string serial_port_name = "/dev/ttyUSB0"; // a good guess on linux


// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const string &msg)
{
  cout << "  " << "ERROR: " << msg << endl;
  
  // NOTE: make a copy of the message if you want to save it or send it to another thread. Do not
  // save or pass the reference itself, it will not be valid after this function returns! 
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
// 'data' depends on mode that was set when creating the RazorAHRS object. In this case 'data'
// holds 3 float values: yaw, pitch and roll.
void on_data(const float data[])
{
  cout << "  " << fixed << setprecision(1) 
  << "Yaw = " << setw(6) << data[0] << "      Pitch = " << setw(6) << data[1] << "      Roll = " << setw(6) << data[2] << endl;

  // NOTE: make a copy of the yaw/pitch/roll data if you want to save it or send it to another
  // thread. Do not save or pass the pointer itself, it will not be valid after this function
  // returns!
  
  // If you created the Razor object using RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
  // instead of RazorAHRS::YAW_PITCH_ROLL, 'data' would contain 9 values that could be printed like this:
  
  // cout << "  " << fixed << setprecision(1)
  // << "ACC = " << setw(6) << data[0] << ", " << setw(6) << data[1] << ", " << setw(6) << data[2]
  // << "        MAG = " << setw(7) << data[3] << ", " << setw(7) << data[4] << ", " << setw(7) << data[5]
  // << "        GYR = " << setw(7) << data[6] << ", " << setw(7) << data[7] << ", " << setw(7) << data[8] << endl;

}

RazorAHRS *razor;
int main()
{
  cout << endl;
  cout << "  " << "Razor AHRS C++ test" << endl;
  cout << "  " << "Press RETURN to connect to tracker. When you're done press RETURN again to quit." << endl;
  getchar();  // wait RETURN
  cout << "  " << "Connecting..." << endl << endl;
  
  try
  {
    // Create Razor AHRS object. Serial I/O will run in background thread and report
    // errors and data updates using the callbacks on_data() and on_error().
    // We want to receive yaw/pitch/roll data. If we wanted the unprocessed raw or calibrated sensor
    // data, we would pass RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
    // instead of RazorAHRS::YAW_PITCH_ROLL.
    razor = new RazorAHRS(serial_port_name, on_data, on_error, RazorAHRS::ACC_MAG_GYR_CALIBRATED);    //YAW_PITCH_ROLL
    
    // NOTE: If these callback functions were members of a class and not global
    // functions, you would have to bind them before passing. Like this:
    
    // class Callback
    // {
    //   public:
    //     void on_data(const float ypr[]) { }
    //     void on_error(const string &msg) { }
    // };
    
    // Callback c;
    
    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, &c, placeholders::_1),
    //    bind(&Callback::on_error, &c, placeholders::_1),
    //    RazorAHRS::YAW_PITCH_ROLL);
    
    // If you're calling from inside of "c" you would of course use "this" instead of "&c".
  }
  catch(runtime_error &e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port in Example.cpp?" << endl;
    return 0;
  }
  
  getchar();  // wait for RETURN key
  return 0;
}

*/
