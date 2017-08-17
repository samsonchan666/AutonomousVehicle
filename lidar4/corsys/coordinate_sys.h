#ifndef COORDINATE_SYS_H
#define COORDINATE_SYS_H

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <vector>

//Grid map size and scale
#define X_SCALE 100
#define Y_SCALE 100
#define MAX_DIS SCALE * X_SCALE
#define DIS_TO_CENTER X_SCALE/2+0.5
#define SCALE 225 //half of width of robot(mm)

using namespace std;

class Point{
private:
	float x, y;
public:
	Point() : x(0), y(0) {}
	Point(float x, float y) { set(x,y);}
	~Point() {}
	void set(float cx, float cy) { x = cx; y = cy;}
	float getX() const { return x;}
	float getY() const { return y;}

};


class Block
{
private:
	bool robotExist;
	vector<Point> pointVec;
public:
	Block() : robotExist(false) {}
	~Block() {}
	void pushPointVec(Point _point){
		pointVec.push_back(_point);
	}

	void setRobotExist(bool exist) { robotExist = exist;} 

	vector<Point> getPointVec() { return pointVec;}
	bool getRobotExist() { return robotExist;}
};

//Grip map with (0,0) at top-left hand corner
class CoordinateSys
{
private:
	Block blocks[X_SCALE][Y_SCALE]; //block(x,y)
public:
	CoordinateSys() {}
	~CoordinateSys() {}

	void assignBlock(Point point){
		int xIndex = 0; int yIndex = 0;

		//Round down to assign int index
		xIndex = point.getX() / SCALE;
		yIndex = point.getY() / SCALE;
		if (xIndex > 100 || yIndex > 100 || xIndex < 0 || yIndex < 0) {
			cout << "Error!! Array out range" << endl;
			return;
		}
		blocks[xIndex][yIndex].pushPointVec(point);
		cout << "x: " << xIndex << "y: " << yIndex << endl;
	}

	//Directly assign a block from lidar's raw data
	void assignBlock(float angle, float distance){
		assignBlock(transformToPoint(angle,distance));
	}

	void assignRobotBlock(int x, int y){
		blocks[x][y].setRobotExist(true);
	}

	void printBlocks(){
		for (int i = 0; i < Y_SCALE; i++){
			for (int j = 0; j < X_SCALE; j++){
				vector<Point> points = blocks[j][i].getPointVec();
				if (blocks[j][i].getRobotExist()) cout << "@" ;
				else if (points.size() >= 1) cout << "#";
				else cout << " ";
				}
			cout << endl;
			}
	}

	int calcuQuadrant(float angle){
		cout << angle << endl;
		if (angle >= 0 && angle <= 90) return 1; 
		else if (angle >=90 && angle <= 180) return 4;
		else if (angle >= 180 && angle <= 270) return 3;
		else if (angle >=270 && angle <=360) return 2;
		else return -1;
	}

	Point transformToPoint(float angle, float distance){
		float distance_x, distance_y;

		//Distance to the robot, IMU may be used to keep up with
		//robot's location
		const float dist_to_lidar = SCALE * DIS_TO_CENTER;	
		
		switch(calcuQuadrant(angle)){
			case 1:
				distance_x = dist_to_lidar + distance * sin(angle);
				distance_y = dist_to_lidar - distance * cos(angle);			
				break;
			case 2:
				distance_x = dist_to_lidar - distance * sin(360-angle);
				distance_y = dist_to_lidar - distance * cos(360-angle);	
				break;
			case 3:
				distance_x = dist_to_lidar - distance * sin(angle-180);
				distance_y = dist_to_lidar + distance * cos(angle-180);	
				break;
			case 4:
				distance_x = dist_to_lidar + distance * sin(180-angle);
				distance_y = dist_to_lidar + distance * cos(180-angle);	
				break;			
			case -1:
				cout << "error" << endl;
				break;
			}
		cout << distance_x << " " << distance_y << endl;
		Point point(distance_x,distance_y);
		return point;
	}

};






#endif