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
#include <wiringPi.h>

#include "./include/rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "./include/coordinate_sys.h"
#include "IMU.h"
#include "./include/motor.h"

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
#define m2mm(i) i*100*10

using namespace rp::standalone::rplidar;

void* run_IMU(void *);
void* run_lidar(void *);
void run_motor(char c );

u_result capture_and_display(RPlidarDriver * drv);
void print_usage(int argc, const char * argv[]);

bool IMU_loop = true;
bool lidar_loop = true;
bool motor_loop = true;
CoordinateSys corSys;

// Set of variables to command what should display
bool get_acc_dis = false;
bool get_avr_acc = false;
bool get_avr_of_avr_acc = false;
bool get_map = false;


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
        
        IMU* imu = new IMU();    //Create the IMU object  
                
        int IMUThreadRes;
        pthread_t IMUThread; //create new thread
        if((IMUThreadRes = pthread_create(&IMUThread,NULL,run_IMU,(void* )imu)))
        {
            printf("Unable to create IMU thread: %d\n\r",IMUThreadRes);
            IMU_loop = false;
        }
        
        int lidarThreadRes;
        pthread_t lidarThread;
        if((lidarThreadRes = pthread_create(&lidarThread,NULL,run_lidar, (void* )drv)))
        {
            printf("Unable to create lidar thread: %d\n\r",lidarThreadRes);
            lidar_loop = false;
        } 

//        int motorThreadRes;
//        pthread_t motorThread;
//        if((motorThreadRes = pthread_create(&motorThread,NULL,run_motor, NULL)))
//        {
//            printf("Unable to create motor thread: %d\n\r", motorThreadRes);
//            motor_loop = false;
//        }        
    while (1){
        printf("Press x to exit\n");
        printf("Press c to clear the window\n");
        printf("Press v to display distance\n");
        printf("Press b to display average acceleration in 1s\n");
        printf("Press n to display average acceleration in 10s\n" );
        printf("Press m to display map\n");
        printf("Press wasd to control the robot, q to stop it\n");
        char c = getchar();
        if (c == 'x') break; //exit
        else if (c == 'c'){  //clear display
            get_acc_dis = false;
            get_avr_acc = false;
            get_avr_of_avr_acc = false;   
            get_map = false;
            system("clear");
        }
        else if (c == 'v') get_acc_dis = true;//get accumulated distance
        else if (c == 'b') get_avr_acc = true;//get average acceleration
        else if (c == 'n') get_avr_of_avr_acc = true;//get average of averages of acceleration
        else if (c == 'm') get_map = true;  // display map from lidar
        else if (c == 'q') run_motor(c);
        else if (c == 'w') run_motor(c);
        else if (c == 'a') run_motor(c);
        else if (c == 's') run_motor(c);
        else if (c == 'd') run_motor(c);
     }
        
    void *status;
    int IMU_JoinRes, lidar_JoinRes, motor_JoinRes;
    IMU_loop = false;
    lidar_loop = false;
//    motor_loop = false;
//    motor_term();
    
    if (IMU_JoinRes=pthread_join(IMUThread, &status)){
        printf("Error:unable to join, %d", IMU_JoinRes );
    }   
    if (lidar_JoinRes=pthread_join(lidarThread, &status)){
        printf("Error:unable to join, %d", lidar_JoinRes );
    }
//    if (motor_JoinRes=pthread_join(motorThread, &status)){
//        printf("Error:unable to join, %d", motor_JoinRes );
//    }   
    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

void* run_IMU(void * _imu){
    ofstream dataLog;
    dataLog.open("log.txt");
    IMU * imu = static_cast<IMU *>(_imu);
    int count = 0; //Offset counts
    int count2 = 0; //Offset counts
    float x_acc_avr =0 ;
    float y_acc_avr =0 ;
    float x_accum_avr_acc = 0 ;
    float y_accum_avr_acc = 0 ;
    float timeInterval = 0.001;  //second 
    float _timeInterval = 0.001 * pow(10, 6); //macro second (10^-6)
    int no_of_avr_sample = 100;
    int no_of_acc_sample = 100;
    Accelerometer cur_acc, pre_acc;
    cur_acc.ax = 0; cur_acc.ay = 0; cur_acc.az  = 0; //Initialize it
    pre_acc.ax = 0; pre_acc.ay = 0; pre_acc.az  = 0; //Initialize it
    float x_cur_vel = 0; float y_cur_vel = 0;
    float x_pre_vel = 0; float y_pre_vel = 0;
    float x_distance = 0;   //accumulate every 0.001s
    float y_distance = 0;   //accumulate every 0.001s
    float x_distance_t = 0; //accumulate every 1s 
    float y_distance_t = 0; //accumulate every 1s 
    
    while (IMU_loop){
//        if (count2 < no_of_avr_sample){
            if (count == no_of_acc_sample){   //0.1s
                x_acc_avr /= no_of_acc_sample;
                y_acc_avr /= no_of_acc_sample;
                x_accum_avr_acc += x_acc_avr;
                y_accum_avr_acc += y_acc_avr;
                        
                if (get_avr_acc) {
                    printf("Offset: %.3f %.3f\n", x_acc_avr, y_acc_avr );               
                    dataLog << x_acc_avr << "\t" << y_acc_avr << "\n";
                }
                
                //Noise removal
                if ( abs(x_acc_avr) > 3)  x_distance_t += x_distance;                                
                if ( abs(y_acc_avr) > 3) y_distance_t += y_distance; 
                
                // reset all the count
                count = 0; 
                y_acc_avr = 0;
                x_acc_avr = 0;
                x_distance = 0;
                y_distance = 0;
                
                if (get_acc_dis){
                    printf("Velocity: %.3f %.3f\n", x_cur_vel, y_cur_vel);
                    printf("Distance(accumulate in second): %.4f %.4f ", x_distance_t, y_distance_t );
                    printf("%d %d \n", int(floor(m2mm(x_distance_t)/(SCALE))), int(floor(m2mm(y_distance_t)/SCALE)));              
                }
                
                int x = X_SCALE/2 - int(floor(m2mm(x_distance_t)/SCALE));
                int y = Y_SCALE/2 - int(floor(m2mm(y_distance_t)/SCALE));
//                corSys.assignRobotBlock(x, y);
                
                count2++;
            }
//        }
//        else {
//            if (get_avr_of_avr_acc) {
//                x_accum_avr_acc /= no_of_avr_sample;
//                y_accum_avr_acc /= no_of_avr_sample;
//                printf("Average of 100 averages samples: %.3f %.3f\n", x_accum_avr_acc, y_accum_avr_acc);
//            }
//            //Reset the value
//            x_accum_avr_acc = 0;
//            y_accum_avr_acc = 0;            
//            count2 = 0; //reset the counter
//        }
 
        if (!(imu->run_sensors())) continue; //Run the sensors, skip extreme value
        cur_acc = imu->getAccelerometer();
        
//      Accumulate acceleration for noise testing
        y_acc_avr += cur_acc.ay;   
        x_acc_avr += cur_acc.ax;   
                  
        x_cur_vel = x_pre_vel + (raw2mssq(cur_acc.ax) * timeInterval);    // v = v0 + at, set the t to 0.001   
        x_distance = x_distance + ( x_cur_vel  * timeInterval);   // s = s0 +vt, set t to 0.001
        
        y_cur_vel = y_pre_vel + (raw2mssq(cur_acc.ay) * timeInterval);
        y_distance = y_distance + ( y_cur_vel  * timeInterval);           
 
        
//        printf("%.3f %.3f\n", x_distance, y_distance);
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
    delay(_word_size_t(4000));
    RPlidarDriver * drv = static_cast<RPlidarDriver *>(_drv);
    drv->startMotor();
     // take only one 360 deg scan and       display the result as a histogram
     ////////////////////////////////////////////////////////////////////////////////
    if (IS_FAIL(drv->startScan( /* true */ ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
    {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        pthread_exit(NULL);
    }
    while (lidar_loop){
        if (get_map){
            if (IS_FAIL(capture_and_display(drv))) {
               fprintf(stderr, "Error, cannot grab scan data.\n");
               break;
           }           
        }
      }
    pthread_exit(NULL);
}

void run_motor(char c){
    double speed = 0.2;
    if (wiringPiSetupGpio() == -1) {
        printf("Could not initialize Wiring PI\n");
    }
    motor_init();
    if (c == 'q') {
//       motor_left_set_normalized_speed(0);
//       motor_right_set_normalized_speed(0);
       motor_term();
    }
    else if (c == 'w') {
       motor_left_set_normalized_speed(speed);
       motor_right_set_normalized_speed(speed);
//       delay(_word_size_t(1000));       
    }
    else if (c == 's') {
       motor_left_set_normalized_speed(-speed);
       motor_right_set_normalized_speed(-speed);
//       delay(_word_size_t(1000));       
    }
    else if (c == 'a') {
       motor_left_set_normalized_speed(speed);
       motor_right_set_normalized_speed(-speed);
//       delay(_word_size_t(1000));       
    }    
    else if (c == 'd') {
       motor_left_set_normalized_speed(-speed);
       motor_right_set_normalized_speed(speed);
//       delay(_word_size_t(1000));       
    }  
//    pthread_exit(NULL);
}

void bug2(int x, int y){    
    int robotX, robotY;
    enum { GOALSEEK, WALLFOLLOW, STOP};
    int state = GOALSEEK;
    //float goalAngle
    while(!corSys.atGoal()){
        
        corSys.getRobotPosition(&robotX, &robotY);
        if (state == GOALSEEK){
            //rotationVel = ComputeGoalSeekRot(goalAngle);
            if( corSys.obstaclesInWay())
            state = WALLFOLLOW;
        }
        else if (state == WALLFOLLOW){
            // rotationVel = ComputeRWFRot(&sonars);
            if( !corSys.obstaclesInWay())
            state = GOALSEEK;
        }
        //robot.SetVelocity(forwardVel, rotationVel);       
    }
    state = STOP;
        //Robot states
//    System.out.println("At Goal!");
//    forwardVel = 0;
//    rotationVel = 0;
//    robot.SetState(DONE);
}  

u_result capture_and_display(RPlidarDriver * drv)
{
    u_result ans;
    
    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    //printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);

        corSys.clearBlocks();
     
            for (int pos = 0; pos < (int)count ; ++pos) {
                 printf("%s theta: %03.2f Dist: %08.2f \n", 
                     (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                     (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                     nodes[pos].distance_q2/4.0f);
                corSys.assignBlock((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f
                    ,nodes[pos].distance_q2/4.0f);
            }

 
            corSys.printBlocks();
        }
    delay(_word_size_t(1000));
    return ans;
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
