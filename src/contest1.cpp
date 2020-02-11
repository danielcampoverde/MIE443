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
double current_time = 0.0;
double previous_time = 0;
int map[20][20];
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
bool final_left_turn = false;
bool final_right_turn = false;
bool compare_done = false;
float left_avg_LaserDist = 0;
float right_avg_LaserDist = 0;
float L = 0.0, LTurn = 0.0, R = 0.0, RTurn = 0.0;
float posX_array[200] = {0.0};
float posY_array[200] = {0.0};
int k = 0;
float current_yaw_sweep, current_yaw_sweepI, current_yaw_Turn, dist_to_sweep = 0;

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
            L = msg->ranges[laser_idx];
            if (isnanf(L))
            {
                L = 1.0;
            }
            left_avg_LaserDist += L;
        }
        for (uint32_t laser_idx = (nLasers / 2) + 15; laser_idx < nLasers / 2 + 27; ++laser_idx)
        {
            R = msg->ranges[laser_idx];
            if (isnanf(R))
            {
                R = 1.0;
            }
            right_avg_LaserDist += R;
        }
        left_avg_LaserDist = left_avg_LaserDist / 12;
        right_avg_LaserDist = right_avg_LaserDist / 12;
        LTurn = msg->ranges[(nLasers / 2) - 20];
        RTurn = msg->ranges[(nLasers / 2) + 20];

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
    ROS_INFO("Position:(%f,%f) Orientation: %f rad or %f degrees. ", posX, posY, yaw, RAD2DEG(yaw));
}

bool sweep(float *yaw, float *angular, float *linear, double *time, float *posX, float *posY, float *posX_array, float *posY_array, int *k)
{
    float F_Dist;
    sweeping = true;

    //pass = true;

    if (!check_yaw)
    {
        *linear = 0.0;
        *angular = 0.0;
        current_time = *time;
        current_yaw_sweep = RAD2DEG(*yaw);
        check_yaw = true;

        *k++;

        ROS_INFO("Sweep Initializing values");
    }
    else if ((*time - current_time) < 6.0)
    {
        *linear = 0.0;
        *angular = M_PI / 9.0;
        ROS_INFO("Sweep initializing");
    }
    else if (abs(int(RAD2DEG(*yaw)) - int(current_yaw_sweep)) >= 3) //int(RAD2DEG(*yaw)) != int(current_yaw
    {
        *angular = M_PI / 9.0;
        *linear = 0.0;
        ROS_INFO("Sweep finishing, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_sweep);
    }
    else //((*time - current_time) > 6. && int(RAD2DEG(*yaw)) == int(current_yaw))
    {
        //done = true;
        sweeping = false;
        check_yaw = false;
        posX_array[*k] = *posX;
        posY_array[*k] = *posY;

        return true;
    }
}
void initial_Sweep(float *yaw, float *angular, float *linear, double *time, float *MidLaserDist, float *minLaserDist)
{
    if (!check_initial)
    {
        current_yaw_sweepI = RAD2DEG(*yaw);
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
        else if ((abs(int(RAD2DEG(*yaw)) - int(current_yaw_sweepI)) >= 3)) //(int(RAD2DEG(*yaw)) != int(current_yaw)) &&
        {
            *angular = M_PI / 9.0;
            *linear = 0.0;
            ROS_INFO("Initial Sweeping, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_sweepI);
        } //360 sweep is done
        else
        {
            first_sweep = true;
        }
    }
    else
    {
        first_sweep = true;
        if (maxYaw < 0)
        {
            if ((abs(int(RAD2DEG(*yaw)) - int(maxYaw)) >= 3)) //(int(RAD2DEG(*yaw)) != int(maxYaw)) &&
            {
                *angular = -M_PI / 7.0;
                *linear = 0.0;
                ROS_INFO("360 done, Sweeping right to maxYaw, yaw:%f, maxYaw:%f", RAD2DEG(*yaw), maxYaw);
            }
            else
            {
                check_first_revolve = true; //first revolve complete
            }
        }
        else if (maxYaw >= 0)
        {
            if ((abs(int(RAD2DEG(*yaw)) - int(maxYaw)) >= 3)) //(int(RAD2DEG(*yaw)) != int(maxYaw)) &&
            {
                *angular = M_PI / 7.0;
                *linear = 0.0;
                ROS_INFO("360 done, Sweeping left to maxYaw, yaw:%f, maxYaw:%f", RAD2DEG(*yaw), maxYaw);
            }
            else
            {
                check_first_revolve = true; //first revolve complete
            }
        }
    }
}
//bool compare complete
void compare_Turn(float *yaw, float *angular, float *linear, float *MidLaserDist, float *LTurn, float *RTurn)
{
    float LDist = 0.0, RDist = 0.0;
    if (!check_compare_Turn)
    {
        current_yaw_Turn = RAD2DEG(*yaw);
        check_compare_Turn = true;
        *linear = 0.0;
        *angular = 0.0;
        final_left_turn = false;

        if (current_yaw_Turn < 0)
        {
            current_yaw_Turn += 360; //current yaw is fixed to this value
        }
    }
    else
    {
        //first turn left and check if the LDist is > 2.0

        if (RAD2DEG(*yaw) < 0)
        {
            *yaw += 360;
        }

        if ((abs(RAD2DEG(*yaw) - (current_yaw_Turn - 25)) > 3.0) && !check_Left_Turn)
        {
            *linear = 0.0;
            *angular = M_PI / 7.0;
            ROS_INFO("Compare: Turning Left");
            ROS_INFO("Initial Sweeping, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_Turn);
        }
        else //bool
        {
            check_Left_Turn = true; //first left turn is now complete
            LDist = *LTurn;
        }
        if (check_Left_Turn && LDist >= 2.0 && !final_left_turn) //bool
        {                                                        //turn left and exit the function
            if ((abs(RAD2DEG(*yaw) - (current_yaw_Turn - 45)) < 3.0) && !final_left_turn)
            {
                *linear = 0.0;
                *angular = M_PI / 7.0;
                ROS_INFO("Compare: Final Left Turn");
                ROS_INFO("Turning Left, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_Turn);
            }
            else
            {
                compare_done = true;        //will go in the main function
                check_compare_Turn = false; //initial boolean
                check_Right_Turn = false;   //exits the function
                final_left_turn = false;
                check_Left_Turn = false;
            }
        }
        //left is blocked, so will check right now
        else if (check_Left_Turn && !final_left_turn && !compare_done)
        {

            if ((abs(RAD2DEG(*yaw) - (current_yaw_Turn + 25)) > 3.0) && !check_Right_Turn) //deleted final right turn from here
            {
                *linear = 0.0;
                *angular = -M_PI / 7.0;
                ROS_INFO("Compare: Final Right Turn");
                ROS_INFO("Turning Right, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_Turn);
            }
            else
            {
                check_Right_Turn = true;
                RDist = *RTurn; //first right turn to check for value has now been complete
            }
            if (check_Right_Turn)
            {
                if (((RDist >= 2.0) || ((RDist - LDist) > 0.65)) && !compare_done)
                { //turn right and exit the function
                    if ((abs(RAD2DEG(*yaw) - (current_yaw_Turn + 45)) < 3.0))
                    {
                        *linear = 0.0;
                        *angular = -M_PI / 7.0;
                        ROS_INFO("Compare: Final Left Turn");
                        ROS_INFO("Turning Left, yaw:%f, current yaw:%f", RAD2DEG(*yaw), current_yaw_Turn);
                    }
                    else
                    {

                        compare_done = true;        //will go in the main function
                        check_compare_Turn = false; //initial boolean
                        check_Right_Turn = false;   //exits the function
                        final_left_turn = false;
                        check_Left_Turn = false;
                    }
                }
                else //if both right and left are blocked then just turn around until mid dist is > 2.0
                {
                    *linear = 0.0;
                    *angular = -M_PI / 7.0;
                    if (*MidLaserDist > 2.0)
                    {
                        //resetting all the variables back to initial state except compare done which will be defaulted in main loop when calling the function

                        compare_done = true;        //will go in the main function
                        check_compare_Turn = false; //initial boolean
                        check_Right_Turn = false;   //exits the function
                        final_left_turn = false;
                        check_Left_Turn = false;
                    }
                }
            }
        }
    }
}

//function for making sweep distance dependent
float sweepDist(int *k, float *posX_array, float *posY_array, float *posX, float *posY)
{
    float distance_sweep = sqrt(pow((*posX - posX_array[*k]), 2) + pow((*posY - posY_array[*k]), 2));
    return distance_sweep;
}

void goStraight(float *ptr_angular, float angular_speed, float *ptr_linear, float linear_speed)
{
    *ptr_linear = linear_speed;
    //*ptr_angular = angular_speed;
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

#define STEER_ANGULAR 0.2

void steer(float &angular, float &curr_yaw, double desired_angle, bool &done)
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
        if (accu_yaw != 0)
        { // done has been forced, reset variables
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

#define K_GO_STRAIGHT 500 // 500 seems to be too much but still stable, 250 might not be enough
void goStraightFeedback(float *angular, float curr_yaw, float ref_angle)
{
    // P controller 
    float error = yawSmallestDifference(curr_yaw, ref_angle);

    if (std::abs(error) > DEG2RAD(0.5)) // error threshold, smaller values considered as noise and ignored
    {
        if (error > 0) // CCW deviation
        {
            *angular = -K_GO_STRAIGHT * std::abs(DEG2RAD(error));
        }
        else
        {
            *angular = K_GO_STRAIGHT * std::abs(DEG2RAD(error));
        }
        ROS_INFO("Feedback control turning");
    }
    else
    { // robot going straight, no change on angular vel
        *angular = 0;
    }
    ROS_INFO("Feedback control ENABLED! Error is: %f degrees", RAD2DEG(error));
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

    for (int i = 0; i < 20; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            map[i][j] = 0;
        }
    }
    map[9][9] = 1;

    int state = 0; // 0 - do nothing, 1 - save ref_angle, 2 - feedback control
    float ref_angle = 0;

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        //ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        //ROS_INFO(" Orientation deg: %f, rad: %f", RAD2DEG(yaw), yaw);

        ROS_INFO("MidLaserDist: %f, RightLaserDist: %f, LeftLaserDist: %f, MinLaserDist: %f, MaxLaserDist: %f ", MidLaserDist, LeftLaserDist, RightLaserDist, minLaserDist, maxLaserDist);
        ROS_INFO("Left_avg_dist:%f, Right_avg_dist:%f", left_avg_LaserDist, right_avg_LaserDist);
        ROS_INFO("Dist between sweeps: %f", sweepDist(&k, posX_array, posY_array, &posX, &posY));
        //ROS_INFO("Position:(%f,%f)", posX, posY);
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
            check_first_revolve = true;
        }
        else if (check_first_revolve)
        {
            if (state == 1){
                // save ref_angle
                ref_angle = yaw;
                state = 2;
                
            }
            if (state == 2){
                goStraightFeedback( &angular, yaw, ref_angle);
            }
            
            ROS_INFO("Ref yaw saved: %f, curr yaw %f", ref_angle, yaw);
            // if (sweeping || ((time > 110.0 && time < 115.0) || (time > 180.0 && time < 185.0) || (time > 260.0 && time < 265.0) || (time > 340.0 && time < 345.0) || (time > 440.0 && time < 445.0)))
            // {
            //     sweep(&yaw, &angular, &linear, &time, &posX, &posY, posX_array, posY_array, &k);
            //     ROS_INFO("Sweeping"); // linear = 0.0;
            //     sweep_check = true;
            // }

            dist_to_sweep = sweepDist(&k, posX_array, posY_array, &posX, &posY);

            if (sweeping || (dist_to_sweep > 1.8 && dist_to_sweep < 1.9))
            {
                sweep(&yaw, &angular, &linear, &time, &posX, &posY, posX_array, posY_array, &k);
                ROS_INFO("Sweeping"); // linear = 0.0;
                sweep_check = true;
                state = 0; // reset state
            }
            else
            {

                if (MidLaserDist < 0.4 || minLaserDist < 0.4) //go back
                {
                    goStraight(&angular, 0.0, &linear, -0.1);
                    ROS_INFO("Backing Up");
                    state = 0; // reset state
                }
                else if (!any_bumper_pressed && MidLaserDist >= 1.8 && minLaserDist >= 0.8)
                {
                    ROS_INFO("Going Straight Fast");
                    goStraight(&angular, 0.0, &linear, 0.22); //fast
                    if (state != 2) state = 1; 
                }
                else if (!any_bumper_pressed && MidLaserDist >= 1.4 && minLaserDist >= 0.7)
                {
                    goStraight(&angular, 0.0, &linear, 0.18); //slow
                    ROS_INFO("Going Straight Slow");
                    if (state != 2) state = 1;

                }
                else if (!any_bumper_pressed && MidLaserDist >= 1.0 && minLaserDist >= 0.7)
                {
                    goStraight(&angular, 0.0, &linear, 0.15); //really slow
                    ROS_INFO("Going Straight Really Slow");
                    if (state != 2) state = 1;
                }

                else if (!compare_done || (MidLaserDist < 1.82)) //if (!pass)
                {
                    compare_done = false;
                    compare_Turn(&yaw, &angular, &linear, &MidLaserDist, &LTurn, &RTurn);
                    ROS_INFO("Compare Turn Function");
                    state = 0; // reset state
                }
            }
        }
        //updatepos();
        //Updating timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    return 0;
}
