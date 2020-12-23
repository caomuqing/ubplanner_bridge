#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

std_msgs::Float64 posex;
std_msgs::Float64 posey;
std_msgs::Float64 posez;
//double posez;
std_msgs::Float64 roll;
std_msgs::Float64 pitch;
std_msgs::Float64 yaw;
sensor_msgs::PointCloud current_tracked_features;
nav_msgs::Odometry odom_data;
geometry_msgs::Pose2D pose;
sensor_msgs::Imu imu_data;
visualization_msgs::Marker line_segments;
sensor_msgs::LaserScan laser_scan;
std::string child_name;
int noneinfpts = 0;
float smoothcost[10];
float smoothcostimg[10];
nav_msgs::Odometry current_visual_odometry;
int totalnumberofpts = 0;
float vall;
geometry_msgs::PoseWithCovarianceStamped odom_pose_;
float visual_var = 1;

float goodpoints = 0;
//cv::Mat input_image_for_tracking_grey_prev;
tf::TransformListener *tran;
geometry_msgs::PoseStamped current_camera_pose;
ros::Time imgbegin;
ros::Time imubegin;
ros::Time odobegin;
ros::Time gpsbegin;
ros::Time odometry_starting_time;
double time_img_since_last_time = 0;
double time_imu_since_last_time = 0;
sensor_msgs::NavSatFix target_GPS;
sensor_msgs::NavSatFix current_GPS_feedback;
nav_msgs::Odometry currentdata;
bool visual_initized = false;
bool gps_initlized = false, compass_initlized = false;
geometry_msgs::QuaternionStamped current_compass_feedback;
std_msgs::UInt8 current_GPS_feedback_health;
std_msgs::UInt8 current_flight_mode, previous_flight_mode;


geometry_msgs::QuaternionStamped    initial_compass_heading;
sensor_msgs::NavSatFix              initial_GPS;
tf::Quaternion initial_q;

//ros::Publisher NTU_internal_path_pub;
//ros::Publisher NTU_internal_odom_pub;
ros::Publisher vins_est_pub;

geometry_msgs::Vector3 st_vel;

nav_msgs::Odometry feedback;
float map_orig_lat_=1.34300, map_orig_lon_=103.68017;
bool transform_fixed=false;
tf::Transform fixed_transform_world_uav;
geometry_msgs::TransformStamped fixed_transform_world_uav_tf2;
static std::string uav_frame_name="local";
static std::string global_frame_name="world";
float fake_odom_initial_x_=0.0f,
      fake_odom_initial_y_=0.0f,
      fake_odom_initial_z_=0.0f,
      fake_odom_initial_yaw_=0.0f;
bool velocity_updated=false;

// void odometry_Callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     visual_initized = true;
//     odobegin = ros::Time::now();
//     currentdata = *msg;
//     odom_pose_.header = currentdata.header;
//     odom_pose_.header.frame_id = uav_frame_name;
//     odom_pose_.pose = currentdata.pose;
//     odom_pose_.pose.pose.orientation.x = currentdata.pose.pose.orientation.x;
//     odom_pose_.pose.pose.orientation.y = currentdata.pose.pose.orientation.y;
//     odom_pose_.pose.pose.orientation.z = currentdata.pose.pose.orientation.z;
//     odom_pose_.pose.pose.orientation.w = currentdata.pose.pose.orientation.w;

// }





double orientation_roll, orientation_pitch, orientation_yaw;
geometry_msgs::QuaternionStamped compass_quaternion;
//geometry_msgs::QuaternionStamped    initial_compass_heading;
//sensor_msgs::NavSatFix              initial_GPS;
void fcc_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr &this_gps)
{
    //if(current_GPS_feedback_health.data == 0)
    current_GPS_feedback = *this_gps;
    if(!gps_initlized)
    {
        if(this_gps->longitude != 0.0 && this_gps->latitude != 0.0)
        {
            initial_GPS = *this_gps;
            gps_initlized = true;
            std::cout << std::setprecision(10) << "gps_initlized" << std::endl;
            gpsbegin = ros::Time::now();
        }
    }

    //std::cout << "\n\n\ncurrent_GPS_feedback\n" << current_GPS_feedback << std::endl;
}


void fcc_orientation_Callback(const geometry_msgs::QuaternionStamped::ConstPtr &this_quaternion)
{

    compass_quaternion=*this_quaternion;

        tf::Quaternion q(
        compass_quaternion.quaternion.x,
        compass_quaternion.quaternion.y,
        compass_quaternion.quaternion.z,
        compass_quaternion.quaternion.w);
    tf::Matrix3x3 m(q);
 
    // tf::Quaternion q_rotate_to_north;
    // q_rotate_to_north.setRPY(0,0,-0.5*3.1415927);
    // tf::Quaternion _q2= q_rotate_to_north*q;
    // tf::Matrix3x3 m(_q2);

    // compass_quaternion.quaternion.x = _q2.x();
    // compass_quaternion.quaternion.y = _q2.y();
    // compass_quaternion.quaternion.z = _q2.z();
    // compass_quaternion.quaternion.w = _q2.w();

    m.getRPY(orientation_roll, orientation_pitch, orientation_yaw);
    std::cout << "roll_orig " << orientation_roll*180/3.1415916 << "pitch_orig " << orientation_pitch*180/3.1415916  
        << " yaw_orig " << orientation_yaw*180/3.1415916 << std::endl;
    
    if(!compass_initlized)
    {   
        compass_initlized = true;
        initial_compass_heading =  compass_quaternion;
        std::cout << "compass_initlized" << std::endl;
    }
}


void st_vel_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    st_vel=msg->vector;
    velocity_updated=true;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "~");

    child_name = uav_frame_name;
    ros::Time::init();
    ros::NodeHandle node;

    if (!node.getParam("fake_odom_initial_yaw", fake_odom_initial_yaw_)||
        !node.getParam("fake_odom_initial_x", fake_odom_initial_x_)||
        !node.getParam("fake_odom_initial_y", fake_odom_initial_y_)||
        !node.getParam("fake_odom_initial_z", fake_odom_initial_z_)){ 
        std::cout<<"not getting param!"<<"\n";
        exit(-1);
    }


    odometry_starting_time = ros::Time::now();


    //ros::Subscriber Visual_odometry_sub = node.subscribe("/vins_estimator/odometry", 1, &odometry_Callback);

    ros::Subscriber fcc_orientation_sub = node.subscribe("/dji_sdk/attitude", 1, &fcc_orientation_Callback);
    ros::Subscriber fcc_gps_sub = node.subscribe("/dji_sdk/gps_position", 1, &fcc_gps_Callback);

    ros::Subscriber st_vel_sub = node.subscribe("/dji_sdk/velocity", 1, &st_vel_cb);


    //NTU_internal_odom_pub= node.advertise<nav_msgs::Odometry>("/NTU_internal/drone_feedback", 1);
    vins_est_pub= node.advertise<nav_msgs::Odometry>("/vins_estimator/odometry", 1);

    current_GPS_feedback_health.data = 1;
    current_flight_mode.data = 99;
    previous_flight_mode.data = 99;
    //ros::spin();
    ros::Rate loop_rate(20);

    //tf::TransformListener lr(ros::Duration(1000));
    //tran = &lr;
    while (ros::ok())
    {

        feedback.pose.pose.position.x = (current_GPS_feedback.longitude -initial_GPS.longitude)*1.1131/0.00001;
        feedback.pose.pose.position.y = (current_GPS_feedback.latitude -initial_GPS.latitude)*1.1131/0.00001;
        feedback.pose.pose.position.z = current_GPS_feedback.altitude - initial_GPS.altitude;
      //  feedback.pose.pose.orientation.z = orientation_yaw - 3.1415926/2;

        feedback.pose.pose.orientation.x = compass_quaternion.quaternion.x;
        feedback.pose.pose.orientation.y = compass_quaternion.quaternion.y;
        feedback.pose.pose.orientation.z = compass_quaternion.quaternion.z;
        feedback.pose.pose.orientation.w = compass_quaternion.quaternion.w;

        // from NEU(seriously?) to ENU frame
        feedback.twist.twist.linear.x = st_vel.x;
        feedback.twist.twist.linear.y = st_vel.y;
        feedback.twist.twist.linear.z = st_vel.z;

        //NTU_internal_odom_pub.publish(feedback);


        if(gps_initlized && compass_initlized && !transform_fixed)
        {
            //fixed_transform_world_uav_tf2.header.seq=msg->header.seq;
            fixed_transform_world_uav_tf2.header.stamp=ros::Time::now();
            fixed_transform_world_uav_tf2.header.frame_id="inter";
            fixed_transform_world_uav_tf2.child_frame_id=uav_frame_name;

            fixed_transform_world_uav_tf2.transform.translation.x=0;
            fixed_transform_world_uav_tf2.transform.translation.y=0;
            fixed_transform_world_uav_tf2.transform.translation.z=0;

            tf::Quaternion q_vins, q_imu;
            q_vins.setRPY(0,0,fake_odom_initial_yaw_/180*3.1415927);
            q_imu.setRPY(3.1415927,0,0);
            tf::Quaternion _ini_quat;
            quaternionMsgToTF(initial_compass_heading.quaternion, _ini_quat);

            tf::Quaternion q_inter_vins= _ini_quat.inverse()*q_vins;

            fixed_transform_world_uav_tf2.transform.rotation.x=q_inter_vins.x();
            fixed_transform_world_uav_tf2.transform.rotation.y=q_inter_vins.y();
            fixed_transform_world_uav_tf2.transform.rotation.z=q_inter_vins.z();
            fixed_transform_world_uav_tf2.transform.rotation.w=q_inter_vins.w();

            transform_fixed=true;
        }

        if (transform_fixed && velocity_updated){
            velocity_updated=false;

            nav_msgs::Odometry fake_vins_odom;
            fake_vins_odom.header.frame_id=uav_frame_name;
            fake_vins_odom.header.stamp=ros::Time::now();
            fake_vins_odom.child_frame_id=uav_frame_name;
            geometry_msgs::PointStamped in_msg, out_msg;
            in_msg.header.frame_id="inter";
            in_msg.point.x=feedback.pose.pose.position.x;
            in_msg.point.y=feedback.pose.pose.position.y;
            in_msg.point.z=feedback.pose.pose.position.z;
            out_msg.header.frame_id=uav_frame_name;
            tf2::doTransform(in_msg, out_msg, fixed_transform_world_uav_tf2);

            fake_vins_odom.pose.pose.position.x=out_msg.point.x+fake_odom_initial_x_;
            fake_vins_odom.pose.pose.position.y=out_msg.point.y+fake_odom_initial_y_;
            fake_vins_odom.pose.pose.position.z=out_msg.point.z+fake_odom_initial_z_;

            tf::Quaternion _q(
            compass_quaternion.quaternion.x,
            compass_quaternion.quaternion.y,
            compass_quaternion.quaternion.z,
            compass_quaternion.quaternion.w);
            tf::Quaternion q_vins, q_imu;
            q_vins.setRPY(0,0,fake_odom_initial_yaw_/180*3.1415927);
            q_imu.setRPY(3.1415927,0,0);
            tf::Quaternion _ini_quat;
            quaternionMsgToTF(initial_compass_heading.quaternion, _ini_quat);

            tf::Quaternion _q_uav=_q*_ini_quat.inverse()*q_vins;

            tf::Matrix3x3 m(_q_uav);
            tf::Matrix3x3 mm(_q);
            double _roll, _pitch, _yaw;            
            m.getRPY(_roll, _pitch, _yaw);

            double _orientation_roll, _orientation_pitch, _orientation_yaw;
            mm.getRPY(_orientation_roll, _orientation_pitch, _orientation_yaw);
            _q_uav.setRPY(_orientation_roll,_orientation_pitch,_yaw);

            _q_uav=_q_uav*q_imu;
            
            fake_vins_odom.pose.pose.orientation.x=_q_uav.x();
            fake_vins_odom.pose.pose.orientation.y=_q_uav.y();
            fake_vins_odom.pose.pose.orientation.z=_q_uav.z();
            fake_vins_odom.pose.pose.orientation.w=_q_uav.w();

            geometry_msgs::Vector3Stamped in_vel, out_vel;
            in_vel.vector=st_vel;
            in_vel.header.frame_id="inter";
            out_vel.header.frame_id=uav_frame_name;
            tf2::doTransform(in_vel, out_vel, fixed_transform_world_uav_tf2);
            fake_vins_odom.twist.twist.linear.x = out_vel.vector.x;
            fake_vins_odom.twist.twist.linear.y = out_vel.vector.y;
            fake_vins_odom.twist.twist.linear.z = out_vel.vector.z;

            vins_est_pub.publish(fake_vins_odom);
            std::cout<< "current position is "<<"\n"<<fake_vins_odom.pose.pose.position<<"\n";
            std::cout<< "current velocity is "<<"\n"<<fake_vins_odom.twist.twist.linear<<"\n";

            std::cout << "roll " << _orientation_roll*180/3.1415916 << " pitch " << _orientation_pitch*180/3.1415916  
                << " yaw " << _yaw*180/3.1415916 << std::endl;
        }

            ros::spinOnce();

            loop_rate.sleep();
    }

    return 0;
};

