#include <ros/ros.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <turtlesim/Pose.h>
//other include
#include <vector>
#include <typeinfo>
#include <stdio.h>
#include <cmath>

using namespace std;

int call_time=0;
// float origin_pose[5];
// float current_pose[5];
int point_count=0; 
float length = 2.0;
float xy_margin = 0.05;
float angle_margin = 0.05;
vector<vector<float> > point;

enum class ActionMode {FORWARD, TURN, STOP};
ActionMode state;

void set_point(float x, float y, float theta){
    for(int i=0 ; i < 4 ; i++){
        int count_x=0;
        int count_y=0;
        if( i%2 == 0){
            count_x = pow(-1,i/2 );
            count_y = 0.0; 
        }
        else{
            count_x = 0.0;
            count_y = pow(-1,i/2 ); 
        }
        float point_theta = theta+M_PI_2*(i+1);
        if (point_theta > M_PI+0.1 ){
            point_theta -= 2*M_PI;
        }
        vector<float> temp;
        x = x+length*count_x;
        y = y+length*count_y;
        temp.push_back(x);
        temp.push_back(y);
        temp.push_back(point_theta);
        point.push_back(temp);
        ROS_INFO("i : %d", i);
        ROS_INFO("pose_x : %f", x);
        ROS_INFO("pose_y : %f", y);
        ROS_INFO("pose_theta : %f", point_theta);

        //wrong write way for 2Dvector
        // point[i].push_back(x+length*count_x);
        // point[i].push_back(y+length*count_y);
        // point[i].push_back(point_theta);
    }
    state = ActionMode::FORWARD;
}

void go_straight(ros::Publisher velocity_pub){
    geometry_msgs::Twist vel;
    vel.linear.x = 1.0 ;
    vel.linear.y = 0.0 ;
    vel.linear.z = 0.0 ;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    velocity_pub.publish(vel);
}

void go_left(ros::Publisher velocity_pub){
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 1.0;
    velocity_pub.publish(vel);
}

void stop(ros::Publisher velocity_pub){
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    velocity_pub.publish(vel);
}

bool reach_goal_xy(float x, float y, ros::Publisher velocity_pub){
    ROS_INFO("point : %f", point[point_count][0]);
    ROS_INFO("x : %f", x);
    ROS_INFO("hihihihihihi : %f", abs(x-point[point_count][0]));
    if( abs(x-point[point_count][0])<=xy_margin && abs(y-point[point_count][1])<=xy_margin){
        ROS_INFO("into into into");
        // stop(velocity_pub);
        state = ActionMode::TURN;
        return true;
    }
    else{
        return false;
    }
}
bool reach_goal_angle(float theta, ros::Publisher velocity_pub){
    if( abs(theta-point[point_count][2])<=angle_margin ){
        ROS_INFO("angleangle");
        point_count+=1;
        state = ActionMode::FORWARD;
        return true;
    }
    else{
        return false;
    }
}

void turtle_callback(const turtlesim::PoseConstPtr &pose, ros::Publisher velocity_pub){
    switch (call_time)
    {
    case 0:
    {
        // origin_pose[0] = pose->x;
        // origin_pose[1] = pose->y;
        // origin_pose[2] = pose->theta;
        // origin_pose[3] = pose->linear_velocity;
        // origin_pose[4] = pose->angular_velocity;
        set_point(pose->x, pose->y, pose->theta);
        call_time = 1;
        break;
    }
    case 1:
    {
        // current_pose[0] = pose.x;
        // current_pose[1] = pose.y;
        // current_pose[2] = pose.theta;
        // current_pose[3] = pose.linear_velocity;
        // current_pose[4] = pose.angular_velocity;
        if ( state==ActionMode::FORWARD && !reach_goal_xy(pose->x , pose->y, velocity_pub) ){
            go_straight(velocity_pub);
        }
        else if( state==ActionMode::TURN && !reach_goal_angle(pose->theta, velocity_pub) ){
            go_left(velocity_pub);
        }
        if(point_count == 4){
            ROS_INFO("nonononononon");
            state = ActionMode::STOP;
            stop(velocity_pub);
        }
        break;
    }
    }
}

int main(int argc, char **argv)
{
    //default
    ros::init(argc, argv, "square");
    ros::NodeHandle n;

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 50);
    ros::Subscriber turtle_pose = n.subscribe<turtlesim::Pose>("turtle1/pose", 20, boost::bind(&turtle_callback, _1, velocity_pub) );
    
    ros::spin();
    return 0;
}