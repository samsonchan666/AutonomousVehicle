/*
 *  RPLIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <fstream>

#include "./include/rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "./include/coordinate_sys.h"
#include "IMU.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

#define GRAVITY 9.80665
#define raw2mssq(i) i*GRAVITY/256

using namespace rp::standalone::rplidar;

void print_usage(int argc, const char * argv[]);
u_result capture_and_display(RPlidarDriver * drv);
// void plot_histogram(rplidar_response_measurement_node_t * nodes, size_t count)
void* run_IMU(void *);
void* run_lidar(void *);

bool IMU_loop = true;
bool lidar_loop = true;

int main(int argc, const char * argv[]) {
    
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    if (argc < 2) {
        print_usage(argc, argv);
        return -1;
    }
    opt_com_path = argv[1];
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_health_t healthinfo;
    rplidar_response_device_info_t devinfo;

    // do {
        // try to connect
        if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
            // break;
        }

        // retrieving the device info
        ////////////////////////////////////////
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) {
            if (op_result == RESULT_OPERATION_TIMEOUT) {
                // you can check the detailed failure reason
                fprintf(stderr, "Error, operation time out.\n");
            } else {
                fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
                // other unexpected result
            }
            // break;
        }

        // print out the device serial number, firmware and hardware version number..
        // printf("RPLIDAR S/N: ");
        // for (int pos = 0; pos < 16 ;++pos) {
        //     printf("%02X", devinfo.serialnum[pos]);
        // }

        // printf("\n"
        //         "Version: "RPLIDAR_SDK_VERSION"\n"
        //         "Firmware Ver: %d.%02d\n"
        //         "Hardware Rev: %d\n"
        //         , devinfo.firmware_version>>8
        //         , devinfo.firmware_version & 0xFF
        //         , (int)devinfo.hardware_version);


        // check the device health
        ////////////////////////////////////////
        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            //printf("RPLidar health status : ");
            switch (healthinfo.status) {
            case RPLIDAR_STATUS_OK:
                printf("OK.");
                break;
            case RPLIDAR_STATUS_WARNING:
                printf("Warning.");
                break;
            case RPLIDAR_STATUS_ERROR:
                printf("Error.");
                break;
            }
           // printf(" (errorcode: %d)\n", healthinfo.error_code);

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            // break;
        }


        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            // break;
        }

        int IMUThreadRes;
        pthread_t IMUThread; //create new thread
        if((IMUThreadRes = pthread_create(&IMUThread,NULL,run_IMU,NULL)))
        {
            printf("Unable to create IMU thread: %d\n\r",IMUThreadRes);
            IMU_loop = false;
        }
        
        int lidarThreadRes;
        pthread_t lidarThread;
        if((lidarThreadRes = pthread_create(&lidarThread,NULL,run_lidar, (void* )drv)))
        {
            printf("Unable to create IMU thread: %d\n\r",lidarThreadRes);
            lidar_loop = false;
        }        
    
    while (1){
        printf("Press any key to to exit ");
         if (getchar()) break;
     }
        
    void *status;
    int IMU_JoinRes, lidar_JoinRes;
    IMU_loop = false;
    lidar_loop = false;
    
    if (IMU_JoinRes=pthread_join(IMUThread, &status)){
        printf("Error:unable to join, %d", IMU_JoinRes );
    }   
    if (lidar_JoinRes=pthread_join(lidarThread, &status)){
        printf("Error:unable to join, %d", lidar_JoinRes );
    }   

    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n"
           "Usage:\n"
           "%s <com port> [baudrate]\n"
           "The default baudrate is 115200. Please refer to the datasheet for details.\n"
           , argv[0]);
}

u_result capture_and_display(RPlidarDriver * drv)
{
    CoordinateSys corSys;
    u_result ans;
    
    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    //printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);
        // plot_histogram(nodes, count);

//        printf("Do you want to see all the data? (y/n) ");
//        int key = getchar();
//        if (key == 'Y' || key == 'y') {
            for (int pos = 0; pos < (int)count ; ++pos) {
//                 printf("%s theta: %03.2f Dist: %08.2f \n", 
//                     (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
//                     (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
//                     nodes[pos].distance_q2/4.0f);
//                corSys.assignBlock((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f
//                    ,nodes[pos].distance_q2/4.0f);
            }
            //corSys.printBlocks();
            //printf("reach here");
        }
//    } else {
//        printf("error code: %x\n", ans);
//    }
    //usleep(1000);
    return ans;
}

void* run_IMU(void * ){
    ofstream dataLog;
    dataLog.open("log.txt");
    IMU imu;    //Create the IMU object  
    int count = 0;
    int count2 = 0; //Offset counts
    float x_acc_avr =0 ;
    float y_acc_avr =0 ;
    float timeInterval = 0.001;  //second 
    float _timeInterval = 0.001 * pow(10, 6); //macro second (10^-6)
    Accelerometer cur_acc, pre_acc;
    pre_acc.ax = 0; pre_acc.ay = 0; pre_acc.az  = 0; //Initialize it
    float x_cur_vel = 0; float y_cur_vel = 0;
    float x_pre_vel = 0; float y_pre_vel = 0;
    float x_distance = 0;   //accumulate every 0.01s
    float y_distance = 0;   //accumulate every 0.01s
    float x_distance_t = 0; //accumulate every 1s 
    float y_distance_t = 0; //accumulate every 1s 
    
    while (IMU_loop){
//        if (count == timeInterval){   //One second
//            if ( abs(x_acc_avr) > 1.2) x_distance_t += x_distance;
//            if ( abs(y_acc_avr) > 1.2) y_distance_t += y_distance;  
//            // reset all the count
//            count = 0; 
//            y_acc_avr = 0;
//            x_acc_avr = 0;
//            printf("%d", count2);
//            count2++;
//            printf("Distance(accumulate in second): %.4f %.4f", x_distance_t, y_distance_t );
//        }          
        if (!(imu.run_sensors())) continue; //Run the sensors, skip extreme value
        cur_acc = imu.getAccelerometer();
        
        //Try to accumulate acceleration for noise testing
        y_acc_avr += cur_acc.ay;   
        x_acc_avr += cur_acc.ax;   
         
        /*
         // Offset test (in raw acceleration value)     
        printf("%d ", count2);
        float no_of_samples = 1000;
        if (count2 > no_of_samples) {
            printf("Offset: %.3f %.3f", x_acc_avr/=no_of_samples, y_acc_avr/no_of_samples );
            break;
        }
        else {
            dataLog << cur_acc.ax << "\t" << cur_acc.ay << "\n";
            y_acc_avr += cur_acc.ay;
            x_acc_avr += cur_acc.ax;
        };
        count2++;
        */
        if (abs(cur_acc.ax-pre_acc.ax > 1)){    // To avoid small noise
            x_cur_vel = x_pre_vel + (raw2mssq(cur_acc.ax) * timeInterval);    // v = v0 + at, set the t to 0.01   
            x_distance = x_distance + ( x_cur_vel  * timeInterval);   // s = s0 +vt, set t to 0.01
        }
        
        if (abs(cur_acc.ay-pre_acc.ay > 1)){    // To avoid small noise
            y_cur_vel = y_pre_vel + (raw2mssq(cur_acc.ay) * timeInterval);
            y_distance = y_distance + ( y_cur_vel  * timeInterval);           
        } 
        printf("%.3f %.3f\n", x_cur_vel, y_cur_vel);
        printf("%.3f %.3f\n", x_distance, y_distance);
        fflush(stdout);
        x_pre_vel = x_cur_vel;
        y_pre_vel = y_cur_vel;
        pre_acc = cur_acc;
        count++;
        usleep(_timeInterval);      
    }
    dataLog.close();
    pthread_exit(NULL);
}

void* run_lidar(void * _drv){
    RPlidarDriver * drv = static_cast<RPlidarDriver *>(_drv);
    drv->startMotor();
     // take only one 360 deg scan and display the result as a histogram
     ////////////////////////////////////////////////////////////////////////////////
    if (IS_FAIL(drv->startScan( /* true */ ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
    {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        pthread_exit(NULL);
    }
    while (lidar_loop){
        if (IS_FAIL(capture_and_display(drv))) {
            fprintf(stderr, "Error, cannot grab scan data.\n");
            break;
        }
        //usleep(100);    // Sleep for 0.1s       
      }
    pthread_exit(NULL);
}

// void plot_histogram(rplidar_response_measurement_node_t * nodes, size_t count)
// {
//     const int BARCOUNT =  75;
//     const int MAXBARHEIGHT = 20;
//     const float ANGLESCALE = 360.0f/BARCOUNT;

//     float histogram[BARCOUNT];
//     for (int pos = 0; pos < _countof(histogram); ++pos) {
//         histogram[pos] = 0.0f;
//     }

//     float max_val = 0;
//     for (int pos =0 ; pos < (int)count; ++pos) {
//         int int_deg = (int)((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f/ANGLESCALE);
//         if (int_deg >= BARCOUNT) int_deg = 0;
//         float cachedd = histogram[int_deg];
//         if (cachedd == 0.0f ) {
//             cachedd = nodes[pos].distance_q2/4.0f;
//         } else {
//             cachedd = (nodes[pos].distance_q2/4.0f + cachedd)/2.0f;
//         }

//         if (cachedd > max_val) max_val = cachedd;
//         histogram[int_deg] = cachedd;
//     }

//     for (int height = 0; height < MAXBARHEIGHT; ++height) {
//         float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
//         for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
//             if (histogram[xpos] >= threshold_h) {
//                 putc('*', stdout);
//             }else {
//                 putc(' ', stdout);
//             }
//         }
//         printf("\n");
//     }
//     for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
//         putc('-', stdout);
//     }
//     printf("\n");
// }
