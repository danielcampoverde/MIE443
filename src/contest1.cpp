#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <math.h>

#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;
float angle_parallel = 0.0;
float maxMidDist, maxYaw, original_yaw, current_yaw, current_mid_dist, diag_dist;
float final_ori = 0.0;
float final_ori_max = 0.0;
float final_ori_min = 0.0;
double current_time;
double previous_time = 0;
int map[20][20] ;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 18;


float minLaserDist = std::numeric_limits<float>::infinity();
float maxLaserDist = std::numeric_limits<float>::infinity();
float MidLaserDist = std::numeric_limits<float>::infinity();
float LeftLaserDist = std::numeric_limits<float>::infinity();
float RightLaserDist = std::numeric_limits<float>::infinity();

bool initial_val_assign = false;
bool check_compare_Turn = false;
bool check_Turn_Around = false;
bool check_Left_Turn = false;
bool done = false;
bool revolution = false;
bool check_revolve = false;
bool sweep_check = false;
bool check_yaw = false;
bool sweep_done = false;
bool check = false;
bool check_initial = false;
bool sweeping = false;
bool first_sweep = false;
bool firstpass = false;
bool check_first_revolve = false;
bool left_Turn_Complete = false;
bool check_Right_Turn = false;
bool right_Turn_Complete = false;
float left_avg_LaserDist = 0;
float right_avg_LaserDist = 0;
float L, R;


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
        for (uint32_t laser_idx = (nLasers / 2) - 27; laser_idx < (nLasers / 2) - 15; ++laser_idx)
        {
            L=msg->ranges[laser_idx];
            if(isnanf(L))
            {
                L=1.0;
            }
            left_avg_LaserDist += L;           
        }
        for (uint32_t laser_idx = (nLasers / 2) + 15 ; laser_idx < nLasers / 2 + 27; ++laser_idx)
        {            
            R=msg->ranges[laser_idx];
            if(isnanf(R))
            {
                R=1.0;
            }
            right_avg_LaserDist += R; 
        }
        left_avg_LaserDist = left_avg_LaserDist/12;
        right_avg_LaserDist = right_avg_LaserDist/12; 

        MidLaserDist = msg->ranges[nLasers / 2]; //average
        LeftLaserDist = msg->ranges[nLasers / 2 - 15];
        RightLaserDist = msg->ranges[nLasers / 2 + 15];
        // ((msg -> ranges[nLasers/2 - 15]) + (msg -> ranges[nLasers/2 - 14])+(msg -> ranges[nLasers/2 - 16]))/3;//average
        // RightLaserDist = ((msg -> ranges[nLasers/2 + 15]) + (msg -> ranges[nLasers/2 + 14])+(msg -> ranges[nLasers/2 + 16]))/3;//average
        // //ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f", MidLaserDist, LeftLaserDist, RightLaserDist);
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

bool sweep(float *yaw, float *angular, float *linear, double *time)
{
    float nearest_yaw;
    sweeping = true;
    //pass = true;

    if (!check_yaw)
    {
        *linear = 0.0;
        *angular = 0.0;
        current_time = *time;
        current_yaw = RAD2DEG(*yaw);
        check_yaw = true;
        ROS_INFO("Sweep Initializing values");
    }
    else if ((*time - current_time) < 6.0)
    {
        *linear = 0.0;
        *angular = M_PI / 9.0;
        ROS_INFO("Sweep initializing");
    }
    else if (abs(int(RAD2DEG(*yaw)) - int(current_yaw))>=3)//int(RAD2DEG(*yaw)) != int(current_yaw
    {
        *angular = M_PI / 9.0;
        *linear = 0.0;
        ROS_INFO("Sweep finishing, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw);
    }
    else //((*time - current_time) > 6. && int(RAD2DEG(*yaw)) == int(current_yaw))
    {
        //done = true;
        sweeping = false;
        check_yaw = false;
    }
}
void initial_Sweep(float *yaw, float *angular, float *linear, double *time, float *MidLaserDist, float *minLaserDist)
{
    if (!check_initial)
    {
        current_yaw = RAD2DEG(*yaw);
        current_time = *time;
        check_initial = true;
        *linear = 0.0;
        *angular = 0.0;
    }
    else if (!first_sweep)
    {
        if (*MidLaserDist > maxMidDist && *MidLaserDist < 10. && *minLaserDist > 1.5 && *minLaserDist < 10)
        {
            maxMidDist = *MidLaserDist;
            maxYaw = RAD2DEG(*yaw);
        }
        if ((*time - current_time) < 6.0)
        {
            *linear = 0.0;
            *angular = M_PI / 9.0;
            ROS_INFO("Initial Sweep Start");
        }
        else if ((abs(int(RAD2DEG(*yaw)) - int(current_yaw)) >= 3))//(int(RAD2DEG(*yaw)) != int(current_yaw)) && 
        {
            *angular = M_PI / 9.0;
            *linear = 0.0;
            ROS_INFO("Initial Sweeping, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw);
        } //360 sweep is done
        else
        {
            first_sweep = true;
        }
    }
    else
    {
        first_sweep = true;
        if ((abs(int(RAD2DEG(*yaw)) - int(maxYaw)) >= 3))//(int(RAD2DEG(*yaw)) != int(maxYaw)) && 
        {
            *angular = M_PI / 9.0;
            *linear = 0.0;
            ROS_INFO("360 done, Sweeping to maxYaw, yaw:%f, maxYaw:%f", RAD2DEG(*yaw), maxYaw);
        }
        else
        {
            check_first_revolve = true; //first revolve complete
        }
    }
}
//bool compare complete
void compare_Turn(float *yaw, float *angular, float *linear, double *time, float *MidLaserDist, float *minLaserDist)
{
    if (!check_compare_Turn)
    {
        current_yaw = RAD2DEG(*yaw);
        current_time = *time;
        check_compare_Turn = true;
        *linear = 0.0;
        *angular = 0.0;
        if (current_yaw < -90)
        {
            current_yaw += 360;
        }
    }
    else
    {
        //first turn 90 left and check if the mid dist is > 1.5

        if (*MidLaserDist > maxMidDist && *MidLaserDist < 10. && *minLaserDist > 0.8 && *minLaserDist < 10)
        {
            maxMidDist = *MidLaserDist;
            maxYaw = RAD2DEG(*yaw);
        }
        if (RAD2DEG(*yaw) != (current_yaw - 90) && !check_Left_Turn)
        {
            *linear = 0.0;
            *angular = M_PI / 9.0;
            ROS_INFO("Compare: Turning Left");
            ROS_INFO("Initial Sweeping, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw);
        }
        else
        {
            check_Left_Turn = true;
        }

        if (check_Left_Turn && (maxMidDist >= 1.5) && !left_Turn_Complete)
        {
            if (RAD2DEG(*yaw) != maxYaw)
            {
                *linear = 0.0;
                *angular = -M_PI / 9.0;
            }
            else
            {
                left_Turn_Complete = true; //left turn is now complete
            }
        }
        else if (check_Left_Turn && maxMidDist < 1.5 && left_Turn_Complete)
        { //will turn right
            if (current_yaw >= 90)
            {
                current_yaw -= 360;
            }
            else
            {
                if ((RAD2DEG(*yaw) != (current_yaw + 90)) && !right_Turn_Complete)
                {
                    *linear = 0.0;
                    *angular = -M_PI / 9.0;
                    ROS_INFO("Compare: Turning Left");
                    ROS_INFO("Initial Sweeping, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw);
                }
                else
                {
                    right_Turn_Complete = true; //right turn is now complete
                }
                if (right_Turn_Complete && (RAD2DEG(*yaw) != maxYaw))
                {
                    *linear = 0.0;
                    *angular = M_PI / 9.0;
                }
                else
                {
                    check_compare_Turn = true; //left turn is now complete
                    
                }
            }
        }
    }
}

void goStraight(float *ptr_angular, float angular_speed, float *ptr_linear, float linear_speed)
{
    *ptr_linear = linear_speed;
    *ptr_angular = angular_speed;
}

void updatepos ()
{

     int x = (int (posX*100))/50;
     int y= (int (posY*100))/50;


        map[9+x][9+y]= 1; 

      if (sweeping)
            {
         
            for (int i=0; i<= 2 ; i++)
            { for (int j=0 ; j<=2; j++)
            {
             map[9+x+i-1][9+y+j-1]=1 ;
       
            }
            
            }
            }
      
     for (int i=0; i<20 ; i++)
     { for (int j =0 ; j<20; j++)
      {
          std::cout << map[i][j] <<" ";
      }
      std::cout << std::endl ; 
     }
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

     for (int i=0; i<20 ; i++)
     { for (int j =0 ; j<20; j++)
      {
          map[i][j]= 0; 
      }
     }
     map[9][9]=1;

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        //ROS_INFO(" Orientation deg: %f, rad: %f", RAD2DEG(yaw), yaw);

        ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f, MinLaserDist: %f, MaxLaserDist: %f ", MidLaserDist, LeftLaserDist, RightLaserDist, minLaserDist, maxLaserDist);

        ROS_INFO("Left_avg_dist:%f, Right_avg_dist:%f", left_avg_LaserDist, right_avg_LaserDist);
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        double time = double(secondsElapsed);
        diag_dist = sqrt((posX * posX) + (posY * posY));

        float current_mid_dist = MidLaserDist;

        if (time < 10.0 || !check_first_revolve)
        {
            initial_Sweep(&yaw, &angular, &linear, &time, &MidLaserDist, &minLaserDist);
            ROS_INFO("Initial Sweeping");
        }
        else if (check_first_revolve)
        {
            if (sweeping || ((time > 110.0 && time < 115.0) || (time > 180.0 && time < 185.0) || (time > 260.0 && time < 265.0) || (time > 340.0 && time < 345.0) || (time > 440.0 && time < 445.0)))
            {
                sweep(&yaw, &angular, &linear, &time);
                ROS_INFO("Sweeping"); // linear = 0.0;
                 sweep_check =true; 
            }
            else
            {

                if (MidLaserDist < 0.55 || minLaserDist<0.51)//go back
                {
                    goStraight(&angular, 0.0, &linear, -0.1);
                    ROS_INFO("Backing Up");
                }
                else if (!any_bumper_pressed && MidLaserDist >= 1.5 && minLaserDist >= 0.7)
                {
                    ROS_INFO("Going Straight Fast");
                   goStraight(&angular, 0.0, &linear, 0.2); //fast
                }
                else if (!any_bumper_pressed && MidLaserDist >= 1.2 && minLaserDist >= 0.7)
                {                    
                     goStraight(&angular, 0.0, &linear, 0.15); //slow
                    ROS_INFO("Going Straight Slow");
                }
                else if (!any_bumper_pressed && MidLaserDist >= 0.8 && minLaserDist >= 0.7)
                {
                    goStraight(&angular, 0.0, &linear, 0.1); //really slow
                    ROS_INFO("Going Straight Really Slow");
                }
                // else if (!any_bumper_pressed && MidLaserDist < 0.8 && ((left_avg_LaserDist)>1.4)&& LeftLaserDist>1.2 && right_avg_LaserDist<0.8)
                // {
                //     while (MidLaserDist < 1.4)
                //     {
                //         ROS_INFO("Rotating Left, Left_avg_dist:%f, Right_avg_dist:%f", left_avg_LaserDist, right_avg_LaserDist);
                //         ros::spinOnce();
                //         vel.angular.z = angular;
                //         vel.linear.x = linear;
                //         vel_pub.publish(vel);
                //         goStraight(&angular, M_PI / 9.0, &linear, 0.0); //turn left
                //     }
                // }
                else if (!any_bumper_pressed && MidLaserDist < 0.7 && ((right_avg_LaserDist>1.0)&& RightLaserDist>1.1 && left_avg_LaserDist<1.0)) //) && !pass))
                {                                                                                                  //turn right
                    while (MidLaserDist < 1.4)
                    {
                        ROS_INFO("Rotating Right, Left_avg_dist:%f, Right_avg_dist:%f", left_avg_LaserDist, right_avg_LaserDist);
                        ros::spinOnce();
                        vel.angular.z = angular;
                        vel.linear.x = linear;
                        vel_pub.publish(vel);
                        goStraight(&angular, -M_PI / 9.0, &linear, 0.0);
                    }
                // }
                
                else if((left_avg_LaserDist<0.8 && right_avg_LaserDist<0.6 && minLaserDist<0.75 && minLaserDist>0.55) && MidLaserDist>0.8){
                    goStraight(&angular, 0.0, &linear, 0.1); //really slow
                    ROS_INFO("Squeezing through");
                }
                else //if (!pass)
                {
                    linear = 0.0;
                    angular = M_PI / 9.0;
                    ROS_INFO("Just rotating Left");
                }
            }
        }
        updatepos();
        //Updating timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    return 0;
}
