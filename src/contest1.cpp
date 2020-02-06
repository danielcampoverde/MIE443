#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

float angular = 0.0;
float liner = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];

float minLaserDist = std::numeric_limits<float>::infinity();
float maxLaserDist = std::numeric_limits<float>::infinity();
float MidLaserDist = std::numeric_limits<float>::infinity();
float LeftLaserDist = std::numeric_limits<float>::infinity();
float RightLaserDist = std::numeric_limits<float>::infinity();
bool initial_val_assign = false;
bool check_Left_Turn = false;
bool check_Turn_Around = false;
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 25;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    //Access using bumper [kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //fill with your code

    minLaserDist = std::numeric_limits<float>::infinity();
    maxLaserDist = std::numeric_limits<float>::infinity();
    MidLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle * M_PI / (180 * msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            
        }
        MidLaserDist = msg->ranges[nLasers / 2];//no need to use the min function
        LeftLaserDist = msg -> ranges[nLasers/2 - 15];
        RightLaserDist = msg -> ranges[nLasers/2 + 15];
        //ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f", MidLaserDist, LeftLaserDist, RightLaserDist);
    }
    else
    {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            

        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position:(%f,%f) Orientation: %f rad or %f degrees. ", posX, posY, yaw, RAD2DEG(yaw));
}

void turnLeft(float *yaw_val, float *angular_val, float *linear_val)
{
    float desired_yaw, current_yaw;
    
    if(initial_val_assign = false)
    {
        current_yaw = *yaw_val;
        initial_val_assign = true;
     
        if(current_yaw> -M_PI/2.0 && current_yaw <= M_PI)//from -90 to +180
        {
            desired_yaw = current_yaw-M_PI/2.0;
        }
        else if(current_yaw <= -M_PI/2.0 && current_yaw >= -M_PI){
            desired_yaw = (current_yaw + M_PI*2.0) - M_PI/2.0;
        }
    }
    if((abs(*yaw_val) - abs(current_yaw)) <= M_PI/2.0)
    {
        *angular_val = 0.0;
        *linear_val = 0.0;
        initial_val_assign = false;
        check_Left_Turn = false;
    }
    else if((abs(*yaw_val) - abs(current_yaw))>M_PI/2.0){
        *angular_val = 0.2;
        *linear_val = 0.0;
    }
}


void turnAround(float *yaw, float *angular, float *linear){
    float nearest_yaw;
    if(*yaw <0 && *yaw>=-M_PI){//0 to -90
        nearest_yaw = -M_PI/2.0;
    }
    else if(*yaw >=0 && *yaw<=M_PI){
        nearest_yaw = M_PI/2;
    }
    if(nearest_yaw<0){
        if(*yaw != nearest_yaw-M_PI/180.0){
            *angular = M_PI/12.0;
            *linear = 0.0;
            ros::spinOnce();
        }
    }
    else if(nearest_yaw>0){
        if(*yaw != nearest_yaw-M_PI/180.0){
            *angular = -M_PI/12.0;
            *linear = 0.0;
            ros::spinOnce();
        }
    }
}


    

void goStraight(float *ptr_angular, float angular_speed, float *ptr_linear, float linear_speed ){
    *ptr_linear = 0.2;
    *ptr_angular = 0.0;    
}
void stopRobot(float *ptr_angular, float *ptr_linear ){
    *ptr_linear = 0.0;
    *ptr_angular = 0.0;    
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    //pointers to the laser distances
    float *ptr_LeftLaserDist = &LeftLaserDist;
    float *ptr_MidLaserDist = &MidLaserDist;
    float *ptr_RightLaserDist = &RightLaserDist;



    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        //fill with your code



        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        //ROS_INFO(" Orientation deg: %f, rad: %f", RAD2DEG(yaw), yaw);

        ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f, MinLaserDist: %f, MaxLaserDist: %f ", MidLaserDist, LeftLaserDist, RightLaserDist, minLaserDist, maxLaserDist);
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }


            
            // if(secondsElapsed % 30 == 0){
               
            //    turnAround(&yaw, &angular, &linear); 
            // }
            if(!any_bumper_pressed && MidLaserDist >=2.0 && minLaserDist>0.75 ){;
                goStraight(&angular, 0.0, &linear, 0.2);//fast
                ROS_INFO("Going Straight");
            }
            if(!any_bumper_pressed && MidLaserDist >=1.3 && minLaserDist>0.6 ){;
                goStraight(&angular, 0.0, &linear, 0.15);//fast
                ROS_INFO("Going Straight");
            }
            
   
            else if(!any_bumper_pressed && MidLaserDist<1.3 && LeftLaserDist >=1.2 && minLaserDist>=0.6)//turn Left
            {
            
                angular= M_PI/12.0;
                linear= 0.14;
            }
            else{
                angular=M_PI/12.0;
                linear=0.0;
            }
            




            //The last thing to do is to update the timer.
        
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            loop_rate.sleep();
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
    
 
    }

    return 0;
}   

