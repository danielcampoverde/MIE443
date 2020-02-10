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

#define STEER_ANGULAR 0.5

double angular = 0.0;
double linear = 0.0;
double yaw = 0.0;
float posX = 0.0, posY = 0.0;

double lastRightDist = 0.0;

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
        MidLaserDist = msg->ranges[nLasers / 2]; //no need to use the min function
        LeftLaserDist = msg->ranges[nLasers / 2 - 15];
        RightLaserDist = msg->ranges[nLasers / 2 + 15];
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

    if (initial_val_assign = false)
    {
        current_yaw = *yaw_val;
        initial_val_assign = true;

        if (current_yaw > -M_PI / 2.0 && current_yaw <= M_PI) //from -90 to +180
        {
            desired_yaw = current_yaw - M_PI / 2.0;
        }
        else if (current_yaw <= -M_PI / 2.0 && current_yaw >= -M_PI)
        {
            desired_yaw = (current_yaw + M_PI * 2.0) - M_PI / 2.0;
        }
    }
    if ((abs(*yaw_val) - abs(current_yaw)) <= M_PI / 2.0)
    {
        *angular_val = 0.0;
        *linear_val = 0.0;
        initial_val_assign = false;
        check_Left_Turn = false;
    }
    else if ((abs(*yaw_val) - abs(current_yaw)) > M_PI / 2.0)
    {
        *angular_val = 0.2;
        *linear_val = 0.0;
    }
}

void turnAround(float *yaw, float *angular, float *linear)
{
    float nearest_yaw;
    if (*yaw < 0 && *yaw >= -M_PI)
    { //0 to -90
        nearest_yaw = -M_PI / 2.0;
    }
    else if (*yaw >= 0 && *yaw <= M_PI)
    {
        nearest_yaw = M_PI / 2;
    }
    if (nearest_yaw < 0)
    {
        if (*yaw != nearest_yaw - M_PI / 180.0)
        {
            *angular = M_PI / 12.0;
            *linear = 0.0;
            ros::spinOnce();
        }
    }
    else if (nearest_yaw > 0)
    {
        if (*yaw != nearest_yaw - M_PI / 180.0)
        {
            *angular = -M_PI / 12.0;
            *linear = 0.0;
            ros::spinOnce();
        }
    }
}

void goRobot(double &linear, double &angular)
{
    angular = 0.0;

    if (MidLaserDist < 0.5)
    {
        ROS_INFO("Wall Detected. STOP");
        linear = 0.0;
    }
    else if (MidLaserDist <= 0.8 && MidLaserDist >= 0.5)
    {
        ROS_INFO("Going Straight: Wall closer");
        linear = 0.1;
    }
    else if (MidLaserDist > 0.8)
    {
        ROS_INFO("Going Straight");
        linear = 0.15;
    }
    else
    {
        linear = 0.0;
        angular = STEER_ANGULAR;
    }
}

void stopRobot(double &linear, double &angular)
{
    linear = 0.0;
    angular = 0.0;
}

double yawSmallestDifference(double yaw_end, double yaw_start)
{

    if (yaw_start == 1000)
    {
        return 0.0;
    }

    double relative_yaw = yaw_end - yaw_start;
    // CW case
    if (relative_yaw > 0)
    {
        if (relative_yaw > DEG2RAD(180))
        {
            relative_yaw -= DEG2RAD(360);
        }
    }
    // CCW case
    else
    {
        if (relative_yaw < -DEG2RAD(180))
        {
            relative_yaw += DEG2RAD(360);
        }
    }

    return relative_yaw;
}
void steer(double &angular, double &curr_yaw, double desired_angle, bool &done)
{
    // CW angles are negative from 0 to -180
    // desired_angle IN RAD
    if (desired_angle == 0)
    {
        ROS_ERROR("Must specifiy an angle not equal to zero");
    }
    static double accu_yaw = 0;
    static double prev_yaw = 1000; // inf

    if (done)
    {
        if (accu_yaw != 0){ // done has been forced, reset variables
            angular = 0;
            accu_yaw = 0;
            prev_yaw = 1000;
        }
        return;
    }

    accu_yaw += std::abs(yawSmallestDifference(curr_yaw, prev_yaw));
    ROS_INFO("accumulated yaw %f", accu_yaw);
    prev_yaw = curr_yaw;

    if (std::abs(accu_yaw) > std::abs(desired_angle))
    {
        ROS_INFO(" Steering DONE");
        angular = 0;
        accu_yaw = 0;
        prev_yaw = 1000;
        done = true;
        return;
    }
    // CW (0 to -180)

    if (desired_angle > 0)
    {
        angular = STEER_ANGULAR;
    }
    // CCW case
    else if (desired_angle < 0)
    {
        angular = -STEER_ANGULAR;
    }
    else
    {
        // desired_angle == 0, do nothing
    }
}

double desiredAbsoluteYaw(double &yaw, double &desired_angle)
{
    // CW angles are negative from 0 to -180 IN DEGREES

    double absolute_desired_yaw;

    absolute_desired_yaw = yaw + desired_angle;
    // CW case
    if (desired_angle < 0)
    {
        if (absolute_desired_yaw < -180)
        {
            absolute_desired_yaw += 360;
        }
    }
    // CCW case
    else
    {
        if (absolute_desired_yaw > 180)
        {
            absolute_desired_yaw -= 360;
        }
    }

    return absolute_desired_yaw;
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

    //pointers to the laser distances
    float *ptr_LeftLaserDist = &LeftLaserDist;
    float *ptr_MidLaserDist = &MidLaserDist;
    float *ptr_RightLaserDist = &RightLaserDist;

    bool done = false;
    int state = 1;
    double angle = 0;
    double rAngle = 30;
    double lAngle = -15;
    int wall_detected = 0;
    double time_stamp = 0;

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        //fill with your code

        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        // ROS_INFO(" Orientation deg: %f, rad: %f", RAD2DEG(yaw), yaw);

        ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f, MinLaserDist: %f, MaxLaserDist: %f ", MidLaserDist, LeftLaserDist, RightLaserDist, minLaserDist, maxLaserDist);
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        switch (state)
        {
        case 1: // wait ros for init
            if (secondsElapsed > 0.1)
            {
                state = 2;
            }
            break; // USE break if want to skip to the next while loop iteration
                   // next statements continue to execute if there is no break
        case 2:    // turn 360 degrees
            if (!done)
            {
                steer(angular, yaw, DEG2RAD(360), done); //change back to 360
            }
            if (done)
            {
                state = 3;
                done = false;
                lastRightDist = RightLaserDist;
            }
            break;
        case 3: // Select movement
            if (any_bumper_pressed)
            {
                ROS_INFO("BUMPER PRESSED");
                stopRobot(linear, angular);
                state = 6;
            }
            else if (std::isnan(MidLaserDist) || std::isnan(RightLaserDist)) // Error in Right Laser -> Turn Left
            {
                angle = rAngle;
                stopRobot(linear, angular);
                state = 5;
            }
            else if (std::isnan(LeftLaserDist)) // Error in Left Laser -> Turn Right
            {
                angle = lAngle;
                stopRobot(linear, angular);
                state = 5;
            }
            else if (MidLaserDist > 0.8) // Go Straight
            {
                state = 4;
            }
            else if (0) // GET CLOSER TO WALL ** FIX ME
            {
                angle = lAngle;
                state = 5;
            }
            else if (MidLaserDist <= 0.8 || RightLaserDist < 1) // Turning Left with Right Wall as REF
            {
                angle = rAngle;
                stopRobot(linear, angular);
                state = 5;
            }

            else
            {
                state = -1;
            }

            break;
        case 4: //Go Straight
            if (!done)
            {
                goRobot(linear, angular);
                state = 3;
            }

            break;
        case 5: // turn

            if (!done)
            {
                steer(angular, yaw, DEG2RAD(angle), done);
            }
            if (done)
            {
                state = 3;
                done = false;
                linear = 0.0;
            }
            break;
        case 6: //BUMPER PRESSED
            linear = -0.15;
            angular = STEER_ANGULAR * 2;
            state = 3;
            break;
        case -1:
            ROS_INFO("waiting");
            break; // do nothing
        default:
            ROS_ERROR("Invalid State");
            break;
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
