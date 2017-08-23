#ifndef COORDINATE_SYS_H
#define COORDINATE_SYS_H

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <vector>

//Grid map size and scale
#define X_SCALE 180
#define Y_SCALE 180
#define MAX_DIS SCALE * X_SCALE
//#define X_DIS_TO_CENTER X_SCALE/2+0.5
#define Y_DIS_TO_CENTER 123  //150, 123
#define X_DIS_TO_CENTER 98 //90.5, 98.5
//#define SCALE 225 //half of width of robot(mm)
#define SCALE 150 //try a smaller scale
//#define SCALE 100 //try a smaller scale
// #define SCALE 80 //try a smaller scale  

#define PI 3.14159265

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
        bool getRobotExist() { return robotExist;}
        
	vector<Point> getPointVec() { return pointVec;}
        
        void clearBlock() { pointVec.clear();}	  
};

//Grip map with (0,0) at top-left hand corner
class CoordinateSys
{
private:
	//Distance to the robot, IMU may be used to keep up with
	//robot's location, Initially at center of grid map 
    float x_dist_to_lidar, y_dist_to_lidar;
    //Robot's coordinate in grid map
    int robot_x, robot_y;
    int goalX, goalY;
    Block blocks[X_SCALE][Y_SCALE]; //block(x,y)
public:
    CoordinateSys() : x_dist_to_lidar(SCALE * X_DIS_TO_CENTER),
                      y_dist_to_lidar(SCALE * Y_DIS_TO_CENTER),
                      robot_x(X_SCALE/2),   
                      robot_y(Y_SCALE/2),
                      goalX(0),
                      goalY(0)
    {  
        //initialize the robot position to center
        assignRobotBlock(robot_x,robot_y);
    }
    ~CoordinateSys() {}

    void assignBlock(Point point){
            int xIndex = 0; int yIndex = 0;

            //Round down to assign int index
            xIndex = point.getX() / SCALE;
            yIndex = point.getY() / SCALE; 

            if (outRange(xIndex, yIndex)) return;

            blocks[xIndex][yIndex].pushPointVec(point);
            //printf("x: %d y: %d\n", xIndex, yIndex);
    }

    //Directly assign a block from lidar's raw data
    void assignBlock(float angle, float distance){
        if (distance == 0) return;
        //printf("Angle: %f Distance: %f ", angle, distance);
        assignBlock(transformToPoint(angle,distance));
    }

    void assignRobotBlock(int x, int y){           
        if (outRange(x, y)) return;
        if (robot_x == x && robot_y == y) return;
        robot_x = x;
        robot_y = y;
    }
        
    void getRobotPosition(int* x, int* y){            
        (*x) = robot_x;
        (*y) = robot_y;
    }
    
    void setGoalPos(int x, int y){
        goalX = x;
        goalY = y;
    }
    
    void getGoalPos(int* x, int* y){
        (*x) = goalX;
        (*y) = goalY;
    }
    
    float getGoalAngle(){
    	int x_dis = abs(robot_x - goalX);
    	int y_dis = abs(robot_y - goalY);
    	float angle = RadianToDegree(atan2(y_dis, x_dis));
        switch(calcuQuadrant()){
        	case 1: return (90 - angle);
        	case 2: return (360 - angle);
        	case 3: return (270 - angle);
        	case 4: return (180 - angle);
        	case -1: cout << "Error" << endl; return 0;
        }
    }
    
    bool atGoal(){
        if (robot_x == goalX && robot_y == goalY) return true;
        else return false;                    
    }

    void printBlocks(){
        for (int i = 0; i < Y_SCALE; i++){
            for (int j = 0; j < X_SCALE; j++){
                vector<Point> points = blocks[j][i].getPointVec();
                if (j == robot_x && i == robot_y) cout << "@" ;
                else if (points.size() >= 1) cout << "#";
                else cout << " ";
            }
            cout << endl;
        }
    }
        
    void clearBlocks(){
        for (int i = 0; i < Y_SCALE; i++){
            for (int j = 0; j < X_SCALE; j++){
                blocks[j][i].clearBlock();                   
            }
        }
	}

    void saveMap(){
        ofstream mapLog;
        mapLog.open("map.txt");	
        for (int i = 0; i < Y_SCALE; i++){
            for (int j = 0; j < X_SCALE; j++){
                vector<Point> points = blocks[j][i].getPointVec();
                if (j == robot_x && i == robot_y) mapLog <<  "@";                
                else if (points.size() >= 1) mapLog << "#";
                else mapLog << " ";
            }
            mapLog << "\n";
        }
        mapLog.close();	    	
    }

    void loadmap(){
        Point point;    //A dummy point
        ifstream mapLog("map.txt");
        char c;
        int x = 0;
        int y = 0;
        while (mapLog.get(c)){
            if (c == '\n') {
                y++;
                x = 0;
            }
            if (c == '#') blocks[x][y].pushPointVec(point); 
            x++;
        }
        mapLog.close();
    }

    bool obstaclesInWay(){
            vector<Point> points;
            switch(calcuQuadrant()){
                    case 1:{
                            //Check the nearby three blocks
                            points = blocks[robot_x][robot_y-1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x+1][robot_y-1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x+1][robot_y].getPointVec();
                            if (points.size() != 0) return true;
                            return false;				
                    }
                    case 2:{
                            points = blocks[robot_x][robot_y+1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x+1][robot_y+1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x+1][robot_y].getPointVec();
                            if (points.size() != 0) return true;
                            return false;					
                    }
                    case 3:{
                            points = blocks[robot_x][robot_y+1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x-1][robot_y+1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x-1][robot_y].getPointVec();
                            if (points.size() != 0) return true;
                            return false;					
                    }
                    case 4:{
                            points = blocks[robot_x][robot_y-1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x-1][robot_y-1].getPointVec();
                            if (points.size() != 0) return true;
                            points = blocks[robot_x-1][robot_y].getPointVec();
                            if (points.size() != 0) return true;
                            return false;					
                    }
            }

    }

    //Calculate the quadrant of goal angle from the robot
    int calcuQuadrant(){	
            if (robot_x < goalX && robot_y > goalY) return 1;
            else if (robot_x < goalX && robot_y < goalY) return 4;
            else if (robot_x > goalX && robot_y < goalY) return 3;
            else if (robot_x > goalX && robot_y < goalY) return 2;
            else return -1;
    }

    int calcuQuadrant(float angle){
            //cout << angle << endl;
            if (angle >= 0 && angle <= 90) return 1; 
            else if (angle >=90 && angle <= 180) return 4;
            else if (angle >= 180 && angle <= 270) return 3;
            else if (angle >=270 && angle <=360) return 2;
            else return -1;
    }
        
    float degreeToRadian(float angle){
        return angle * PI / 180.0;
    }

    float RadianToDegree(float angle){
	    return angle * 180.0 / PI;
    }

	Point transformToPoint(float angle, float distance){
		float distance_x, distance_y;
                
		switch(calcuQuadrant(angle)){
			case 1:
				distance_x = x_dist_to_lidar + distance * sin(degreeToRadian(angle));
				distance_y = y_dist_to_lidar - distance * cos(degreeToRadian(angle));			
				break;
			case 2:
				distance_x = x_dist_to_lidar - distance * sin(degreeToRadian(360-angle));
				distance_y = y_dist_to_lidar - distance * cos(degreeToRadian(360-angle));	
				break;
			case 3:
				distance_x = x_dist_to_lidar - distance * cos(degreeToRadian(270-angle));
				distance_y = y_dist_to_lidar + distance * sin(degreeToRadian(270-angle));                               
				break;
			case 4:
				distance_x = x_dist_to_lidar + distance * sin(degreeToRadian(180-angle));
				distance_y = y_dist_to_lidar + distance * cos(degreeToRadian(180-angle));	                             
				break;			
			case -1:
				cout << "error" << endl;
				break;
			}
                //printf("%d XDis: %f YDis: %f ", calcuQuadrant(angle), distance_x, distance_y);
		Point point(distance_x,distance_y);
		return point;
	}
        
        bool outRange(int x, int y){
            if (x < 0 || y < 0) {
                // printf("Error, array index negative\n");
                return true;
            }
            else if (x >= X_SCALE || y >= Y_SCALE){
                // printf("Error, array index outrange\n");
                return true;
            }
            return false;
        }

};

#endif