/*
 * Stopper.h
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#ifndef WANDER_BOT_SRC_STOPPER_H_
#define WANDER_BOT_SRC_STOPPER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <stack>          // std::stack
#include <vector>
//#include <string>

struct task{
  int id;
  double distance;
  std::vector<int> requeriments;
  bool done;
  task(){
    id=0;
    distance=0;
    done=0;
  }
  ~task(){
    requeriments.clear();
  }
  task & operator=(const task & t){
    if(this!=&t){
      id=t.id;
      distance=t.distance;
      done=t.done;
      requeriments.clear();
      requeriments=t.requeriments;
    }
    return *this;
  }
};

class Stopper {
public:
    // Tunable parameters
    const static double FORWARD_SPEED = 0.3;
    const static double ANGULAR_SPEED = 0.3;
    const static double MIN_SCAN_ANGLE = -30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE = +30.0/180*M_PI;
    const static float MIN_DIST_FROM_OBSTACLE = 0.8; // Should be smaller than sensor_msgs::LaserScan::range_max


    Stopper();
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    bool keepMoving; // Indicates whether the robot should continue moving
    bool mask[2]; //Status mask

    std::stack<task> tasks; //Stack of task to do
    std::vector<task> mainTasks; //Main tasks move foward, rotate left and right
    task think();

    int lastTask;
    //Messages to display on screen
    //std::string message;
    //std::string lastMessage;

    void moveForward();
    void rotateLeft();
    void rotateRight();

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif /* WANDER_BOT_SRC_STOPPER_H_ */
