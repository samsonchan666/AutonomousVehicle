#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include "coordinate_sys.h"

using namespace std;

// int calcuQuadrant(float angle){
// 	cout << angle << endl;
// 	if (angle >= 0 && angle <= 90) return 1; 
// 	else if (angle >=90 && angle <= 180) return 4;
// 	else if (angle >= 180 && angle <= 270) return 3;
// 	else if (angle >=270 && angle <=360) return 2;
// 	else return -1;
// }

// Point transformToPoint(float angle, float distance){
// 	float distance_x, distance_y;

// 	//Distance to the robot, IMU may be used to keep up with
// 	//robot's location
// 	const float dist_to_lidar = SCALE * DIS_TO_CENTER;	
	
// 	switch(calcuQuadrant(angle)){
// 		case 1:
// 			distance_x = dist_to_lidar + distance * sin(angle);
// 			distance_y = dist_to_lidar - distance * cos(angle);			
// 			break;
// 		case 2:
// 			distance_x = dist_to_lidar - distance * sin(360-angle);
// 			distance_y = dist_to_lidar - distance * cos(360-angle);	
// 			break;
// 		case 3:
// 			distance_x = dist_to_lidar - distance * sin(angle-180);
// 			distance_y = dist_to_lidar + distance * cos(angle-180);	
// 			break;
// 		case 4:
// 			distance_x = dist_to_lidar + distance * sin(180-angle);
// 			distance_y = dist_to_lidar + distance * cos(180-angle);	
// 			break;			
// 		case -1:
// 			cout << "error" << endl;
// 			break;
// 	}
// 	cout << distance_x << " " << distance_y << endl;
// 	Point point(distance_x,distance_y);
// 	return point;
// }

struct lidarOutput
{
	float distance;
	float angle;
};

int main(){
	CoordinateSys corSys;
	lidarOutput o1;
	o1.distance = 3181.980;
	o1.angle = 45;
	corSys.assignRobotBlock(50,50);
	corSys.assignBlock(o1.angle,o1.distance);
	corSys.assignBlock(o1.angle+90,o1.distance);
	corSys.assignBlock(o1.angle+180,o1.distance);
	corSys.assignBlock(o1.angle+270,o1.distance);
	corSys.printBlocks();
	return 0;
}