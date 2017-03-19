/*
 * Stopper.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#include "Stopper.h"
#include "geometry_msgs/Twist.h"

Stopper::Stopper()
{
  //  keepMoving = true;
  //  keepRotating = false;
    //direction = 0;

    mask[0]=0; //Obstacle mask
    mask[1]=0; //Direction mask
    lastTask=0;
    //mask[2]=0 future masks
    //mask[..]=0;

    //There are three main tasks for the moment
    mainTasks.resize(3);
    //Move foward
      mainTasks[0].id=0,mainTasks[0].distance=100,mainTasks[0].done=0;
      mainTasks[0].requeriments.push_back(0);//Add requeriment if obstacle(0)
    //rotateLeft
      mainTasks[1].id=1,mainTasks[1].distance=0.6,mainTasks[1].done=0;
    //mainTasks[1].requeriments.push_back(0);//Add requeriment if obstacle(0)
    //rotateRight
      mainTasks[2].id=2,mainTasks[2].distance=0.6,mainTasks[2].done=0;
    //_______________________________________________//

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

// Send a velocity command
void Stopper::moveForward() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}
void Stopper::rotateLeft(){
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.angular.z = ANGULAR_SPEED;
  commandPub.publish(msg);

}
void Stopper::rotateRight(){
  geometry_msgs::Twist msg; // The default constructor will set all commands to 0
  msg.angular.z = -ANGULAR_SPEED;
  commandPub.publish(msg);

}

// Process the incoming laser scan message
void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	bool isObstacleInFront = false;

    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int meanIndex = floor((maxIndex+minIndex)/2);

    float maxRange = scan->range_max;
    float meanDistanceLeft=0;
    float meanDistanceRight=0;
    float curr=0;
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
        	isObstacleInFront = true;
        }
        curr = scan->ranges[currIndex]!=scan->ranges[currIndex] ? 0 : scan->ranges[currIndex];
        currIndex<meanIndex ? meanDistanceLeft+= curr : meanDistanceRight+=curr;
    }

    if(isObstacleInFront){
      mask[0]=1;
      mask[1]= meanDistanceLeft>meanDistanceRight ? 0 : 1;
      ROS_INFO("LEFT distance %f",meanDistanceLeft);
      ROS_INFO("RIGHT distance %f",meanDistanceRight);

      //ROS_INFO("Obstacle in front!");
    }


}

task Stopper::think(){ //Analizar entorno para decidir
  //If there arent tasks ont the queue we add the move task//
  if(tasks.size()==0)
    tasks.push(mainTasks[0]);
//_________________________________________________________//
//  ROS_INFO("Numero de tareas: %d",(int)tasks.size());

//If the top task is done then we delete it  //
  if(tasks.top().distance<=0 || tasks.top().done){
      tasks.pop();
  }
//_______________________________________________//
//Check the requeriments of the actual task, if a none of the requeriments is triggered in the mask then we keep doing the task else we made a new task //
  bool fail=0;
  for(int i=0;i<tasks.top().requeriments.size();i++){
    mask[tasks.top().requeriments[i]]!=1 ? :fail=1;
  }
  if(!fail)
    return tasks.top();
  //_______________________________________________________________________________________________________________________________________________________//

//if we reach this point we have to make a new task watching the activated flags on the mask//
//To do it he have a decision tree
// mask[0]==0 (obstacle) ? keep moving foward(task=0);
//else rotate{
//mask[1]==0 (Left) ? Rotate to left(task=1);
//mask[1]==1 (Right) ? Rotate to Right(task=2);
//}
    task t;
    t=mainTasks[0];
    if(mask[0]){
      lastTask==0 ? : mask[1]=((lastTask+1)%2);
      t= !mask[1] ? mainTasks[1] : mainTasks[2];
      mask[0]=0;
    }
    tasks.push(t);
  return t;
  //______________________________________________________________________________________//
}

void Stopper::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");
    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
        task t=think();
        switch (t.id) {
          case 0: //Avanzar
            moveForward();
            ROS_INFO("Moving foward");
          break;
          case 1: //Girar Izq
            rotateLeft();
            t.distance-=ANGULAR_SPEED;
            ROS_INFO("Rotating Left %f", t.distance);
          break;
          case 2: //Girar Der
            rotateRight();
            t.distance-=ANGULAR_SPEED;
            ROS_INFO("Rotating Right %f", t.distance);
          break;
        }
        lastTask=t.id;
        tasks.pop();
        tasks.push(t);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        rate.sleep();
    }
}
