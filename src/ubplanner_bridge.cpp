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


#include <dji_sdk/dji_sdk.h>
#include "unistd.h"

#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>

#include <dji_sdk/SetLocalPosRef.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <mav_msgs/RollPitchYawrateThrust.h> //mq

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float32.h>

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


//geometry_msgs::QuaternionStamped    initial_compass_heading;
sensor_msgs::NavSatFix              initial_GPS;

ros::Publisher NTU_internal_path_pub;
ros::Publisher NTU_internal_odom_pub;

ros::Publisher setpoint_pub;
geometry_msgs::Vector3 st_vel;

/***************adapted from dji demo flight control****************/
ros::Publisher ctrlVelYawRatePub;
ros::ServiceClient      sdk_ctrl_authority_service;
int flight_control_mode = 0, Software_bypass = 0;
int previousSoftware_bypass;
int previousflight_control_mode;
sensor_msgs::Joy input_rc;
dji_sdk::SDKControlAuthority sdkAuthority;
geometry_msgs::Twist Joy_stick_input_in_twist, prev_Joy_stick_input_in_twist;
double pwrfactor=1.3;
double Manual_degree_limit=0.3, Yaw_Manual_degree_limit=0.3, throttle_out_limit=0.25;
/***************adapted from dji demo flight control****************/
ros::Time traj_update_time;
ros::Publisher traj_true_pub;
geometry_msgs::PoseWithCovarianceStamped target_pose_;
bool there_is_pilot_input_=false;
ros::Time pilot_input_time_;

nav_msgs::Odometry feedback;
float map_orig_lat_=1.34300, map_orig_lon_=103.68017;
bool transform_fixed=false;
bool use_gps_=false;
tf::Transform fixed_transform_world_uav;
geometry_msgs::TransformStamped fixed_transform_world_uav_tf2;
static std::string uav_frame_name="local";
static std::string global_frame_name="world";
float height_=0, altitude_offset_=0, manual_throttle_low_=0, manual_throttle_high_=1.0;
nav_msgs::Path current_path_map_frame_;
bool got_path_=false;
std::string sim_type_;
bool manual_enable_,thrust_control_=true, idle_state_before_mission_=true;

void odometry_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    visual_initized = true;
    odobegin = ros::Time::now();
    currentdata = *msg;
    odom_pose_.header = currentdata.header;
    odom_pose_.header.frame_id = uav_frame_name;
    odom_pose_.pose = currentdata.pose;
    // odom_pose_.pose.pose.orientation.x = currentdata.pose.pose.orientation.x;
    // odom_pose_.pose.pose.orientation.y = currentdata.pose.pose.orientation.y;
    // odom_pose_.pose.pose.orientation.z = currentdata.pose.pose.orientation.z;
    // odom_pose_.pose.pose.orientation.w = currentdata.pose.pose.orientation.w;
    if (odom_pose_.pose.pose.position.z>0.15){
        idle_state_before_mission_=false;
    }
    // if(compass_initlized && gps_initlized)
    // {
    //     try
    //     {
    //         geometry_msgs::PointStamped corners1;
    //         corners1.header.frame_id = "camera";
    //         corners1.header.stamp = ros::Time(0);
    //         corners1.point.x = 0;
    //         corners1.point.y = 0;
    //         corners1.point.z = 0;

    //         geometry_msgs::PointStamped cornersvel_points;
    //         tran->transformPoint("far_field", corners1, cornersvel_points);

    //         // copy current_GPS_feedback here
    //         // change this line
    //         //  target_GPS.longitude = current_GPS_feedback.longitude + cornersvel_points.point.x * 0.00001 / 1.1132;
    //         // into
    //         target_GPS.longitude = current_GPS_feedback.longitude - cornersvel_points.point.x * 0.00001 / 1.1132;
    //         target_GPS.latitude = current_GPS_feedback.latitude + cornersvel_points.point.y * 0.00001 / 1.1132;

    //         //     target_GPS.longitude = 103.787625122 + cornersvel_points.point.x * 0.00001 / 1.1132;
    //         //     target_GPS.latitude = 1.2934832484 + cornersvel_points.point.y * 0.00001 / 1.1132;
    //         std::cout << std::fixed;
    //         std::cout << std::setprecision(10);
    //         std::cout << target_GPS.latitude << " " << target_GPS.longitude << std::endl;
    //     }
    //     catch (tf::ExtrapolationException e)
    //     {
    //         std::cout << "Link not updated, Droping current info" << std::endl;
    //     }
    // }



    // current_pose.pose.covariance = { visual_var, 0.0, 0.0, 0.0, 0.0, 0.0,
    //                                  0.0, visual_var, 0.0, 0.0, 0.0, 0.0,
    //                                  0.0, 0.0, visual_var, 0.0, 0.0, 0.0,
    //                                  0.0, 0.0, 0.0, visual_var, 0.0, 0.0,
    //                                  0.0, 0.0, 0.0, 0.0, visual_var, 0.0,
    //                                  0.0, 0.0, 0.0, 0.0, 0.0, visual_var
    //                                };


}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu1)
{
    imubegin = ros::Time::now();

    //imu_data = *imu1;
}




double orientation_roll, orientation_pitch, orientation_yaw;
geometry_msgs::QuaternionStamped compass_quaternion;
//geometry_msgs::QuaternionStamped    initial_compass_heading;
//sensor_msgs::NavSatFix              initial_GPS;
void fcc_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr &this_gps)
{
    //if(current_GPS_feedback_health.data == 0)
    current_GPS_feedback = *this_gps;

    //std::cout << "\n\n\ncurrent_GPS_feedback\n" << current_GPS_feedback << std::endl;
}

void origin_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr &this_gps)
{
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

    // tf::Quaternion q_rotate_to_north;
    // q_rotate_to_north.setRPY(0,0,-0.5*3.1415927);
    // tf::Quaternion _q2= q_rotate_to_north*q;
    // tf::Matrix3x3 m(_q2);

    // compass_quaternion.quaternion.x = _q2.x();
    // compass_quaternion.quaternion.y = _q2.y();
    // compass_quaternion.quaternion.z = _q2.z();
    // compass_quaternion.quaternion.w = _q2.w();
    tf::Matrix3x3 m(q);
    // tf::Quaternion q(
    //     compass_quaternion.quaternion.y,
    //     compass_quaternion.quaternion.x,
    //     -compass_quaternion.quaternion.z,
    //     compass_quaternion.quaternion.w);
    //tf::Matrix3x3 m(q);
 
    m.getRPY(orientation_roll, orientation_pitch, orientation_yaw);
    // std::cout << "roll " << orientation_roll*180/3.1415916 << " pitch " << orientation_pitch*180/3.1415916  
    //     << " yaw " << orientation_yaw*180/3.1415916 << std::endl;
    
    if(!compass_initlized)
    {
        compass_initlized = true;
        //initial_compass_heading =  *this_quaternion;
        std::cout << "compass_initlized" << std::endl;
    }
}

void path_in_ubplanner_Callback(const nav_msgs::Path::ConstPtr &this_path)
{
    current_path_map_frame_ = *this_path;
    got_path_=true;
    // if (!transform_fixed){
    //     ROS_WARN("not getting transform yet! cannot generate waypoint in local frame");
    //     return;
    // } else {
    //     //std::cout << "\n\ncurrent_path_wgs84\n" << current_path_wgs84.poses << std::endl;
    //     nav_msgs::Path current_path_local_ref; 
    //     current_path_local_ref.header.stamp=ros::Time::now();
    //     current_path_local_ref.header.frame_id=uav_frame_name;
    //     for( int i =0; i< current_path_map_frame_.poses.size(); i++)
    //     {
    //         geometry_msgs::PoseStamped path_pose_local_ref;
    //         path_pose_local_ref.header.frame_id=uav_frame_name;
    //         tf2::doTransform(current_path_map_frame_.poses[i], path_pose_local_ref, fixed_transform_world_uav_tf2);
    //         current_path_local_ref.poses.push_back(path_pose_local_ref);

    //     }
    //     // std::cout << "\n\ncurrent_path_map frame\n" << current_path_map_frame_ << std::endl;
    //     // std::cout << "\ncurrent_path_local frame\n" << current_path_local_ref << std::endl;

    //     NTU_internal_path_pub.publish(current_path_local_ref);

    // }

}

void st_vel_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    st_vel=msg->vector;
}

void fix_transform_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{   
    if (visual_initized && gps_initlized){

        tf::Quaternion _q, q_word_local, q_drone_local;
        geometry_msgs::Quaternion q_in1, q_in2;
        q_in1 = compass_quaternion.quaternion;
        q_in2 = odom_pose_.pose.pose.orientation;
        quaternionMsgToTF(q_in1, q_word_local);
        quaternionMsgToTF(q_in2, q_drone_local);

        tf::Matrix3x3 _m(q_drone_local);
        tf::Matrix3x3 _mmm(q_word_local);

        double r,p,y;
        _m.getRPY(r, p, y);
        double _r,_p,_y;
        _mmm.getRPY(_r, _p, _y);
        q_word_local.setRPY(0,0,_y);

        tf::Quaternion q_in22;
        q_in22.setRPY(0,0,y);
        //q.setRPY(odom_data.pose.pose.orientation.x,odom_data.pose.pose.orientation.y, pose.theta);
        _q=q_in22.inverse()*q_word_local;
        fixed_transform_world_uav.setRotation(_q);
        transform_fixed=true;        



        geometry_msgs::TransformStamped trans_tmp;
        trans_tmp.transform.translation.x=0;
        trans_tmp.transform.translation.y=0;
        trans_tmp.transform.translation.z=0;

        tf::Quaternion _q_inv=_q.inverse();
        trans_tmp.transform.rotation.x=_q.x();
        trans_tmp.transform.rotation.y=_q.y();
        trans_tmp.transform.rotation.z=_q.z();
        trans_tmp.transform.rotation.w=_q.w();
        geometry_msgs::PointStamped _inpoint, _outpoint;
        _inpoint.point.x=odom_pose_.pose.pose.position.x;
        _inpoint.point.y=odom_pose_.pose.pose.position.y;
        _inpoint.point.z=odom_pose_.pose.pose.position.z;
        tf2::doTransform(_inpoint, _outpoint, trans_tmp);

        fixed_transform_world_uav.setOrigin(
            tf::Vector3(feedback.pose.pose.position.x-_outpoint.point.x,
                        feedback.pose.pose.position.y-_outpoint.point.y, 
                        feedback.pose.pose.position.z-_outpoint.point.z));

        std::cout<<"in point: "<< _inpoint<<'\n';
        std::cout<<"out point: "<< _outpoint<<'\n';
        std::cout<<"transform is: "<<feedback.pose.pose.position.x-_outpoint.point.x<<'\n'<<
                        feedback.pose.pose.position.y-_outpoint.point.y<<'\n'<<
                        feedback.pose.pose.position.z-_outpoint.point.z<<'\n';
        std::cout<<"rotation is: "<< _q.x()<<"  "<< _q.y()<<"  "<< _q.z()<<"  "<< _q.w()<<'\n';

        fixed_transform_world_uav_tf2.header.seq=msg->header.seq;
        fixed_transform_world_uav_tf2.header.stamp=ros::Time::now();
        fixed_transform_world_uav_tf2.header.frame_id=global_frame_name;

        trans_tmp.transform.rotation.x=_q_inv.x();
        trans_tmp.transform.rotation.y=_q_inv.y();
        trans_tmp.transform.rotation.z=_q_inv.z();
        trans_tmp.transform.rotation.w=_q_inv.w();
        //geometry_msgs::PointStamped _inpoint, _outpoint;
        _inpoint.point.x=feedback.pose.pose.position.x;
        _inpoint.point.y=feedback.pose.pose.position.y;
        _inpoint.point.z=feedback.pose.pose.position.z;
        tf2::doTransform(_inpoint, _outpoint, trans_tmp);

        fixed_transform_world_uav_tf2.transform.translation.x=
          -_outpoint.point.x + odom_pose_.pose.pose.position.x;
        fixed_transform_world_uav_tf2.transform.translation.y=
          -_outpoint.point.y + odom_pose_.pose.pose.position.y;
        fixed_transform_world_uav_tf2.transform.translation.z=
          -_outpoint.point.z + odom_pose_.pose.pose.position.z;

        fixed_transform_world_uav_tf2.transform.rotation.x=_q_inv.x();
        fixed_transform_world_uav_tf2.transform.rotation.y=_q_inv.y();
        fixed_transform_world_uav_tf2.transform.rotation.z=_q_inv.z();
        fixed_transform_world_uav_tf2.transform.rotation.w=_q_inv.w();

        altitude_offset_=height_-odom_pose_.pose.pose.position.z;

    } else  ROS_WARN("GPS or odom not initialised, not setting offset!");

}



void commandRollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg) //mq
{
  if(!Software_bypass||there_is_pilot_input_){
    return;
  }
  //traj_update_time=ros::Time::now();

  if (sim_type_=="vins_dji"||(sim_type_=="vinsfusion_dji_mini"&&!thrust_control_)){
        uint8_t flag;
      ROS_INFO_STREAM_ONCE("Received first roll pitch yawrate altitude command msg");

      double roll_cmd = msg->roll;
      double pitch_cmd = msg->pitch;
      double yaw_rate_cmd = msg->yaw_rate;
      //double alt_cmd = msg->thrust.z+altitude_offset_;
      double vel_cmd; //use vel command instead
      // if(alt_cmd < -20){
      //   ROS_WARN("Altitude command is below minimum.. ");
      //   return;
      // }
      // if(alt_cmd > 60){
      //   ROS_WARN("Altitude command is too high.. ");
      //   return;
      // }
      if (idle_state_before_mission_){
        vel_cmd = 5.0;
        flag = (DJISDK::VERTICAL_THRUST |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_RATE |
                DJISDK::HORIZONTAL_BODY     ///|
                // DJISDK::STABLE_ENABLE
               );       
      } else {
        vel_cmd = msg->thrust.z;
        flag  = (DJISDK::VERTICAL_VELOCITY |
                    DJISDK::HORIZONTAL_ANGLE |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_BODY     ///|
                    // DJISDK::STABLE_ENABLE
                   );       
          if(vel_cmd < -2){
            ROS_WARN("vel command is below minimum.. ");
            vel_cmd = -2;
          }
          if(vel_cmd > 2){
            ROS_WARN("vel command is too high.. ");
            vel_cmd = 2;
          }
     }


    sensor_msgs::Joy Joy_output_msg;
    Joy_output_msg.header.stamp=ros::Time::now();
    Joy_output_msg.axes.push_back(roll_cmd);
    Joy_output_msg.axes.push_back(pitch_cmd);
    Joy_output_msg.axes.push_back(vel_cmd); //use vel command instead
    Joy_output_msg.axes.push_back(yaw_rate_cmd);
    Joy_output_msg.axes.push_back(flag);
    ctrlVelYawRatePub.publish(Joy_output_msg);   

  } else if (sim_type_=="vinsfusion_dji_mini" || sim_type_=="vicon_dji_mini"){
    uint8_t flag = (DJISDK::VERTICAL_THRUST |
                    DJISDK::HORIZONTAL_ANGLE |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_BODY     ///|
                    // DJISDK::STABLE_ENABLE
                   );
      ROS_INFO_STREAM_ONCE("Received first roll pitch yawrate thrust command msg");

      double roll_cmd = msg->roll;
      double pitch_cmd = msg->pitch;
      double yaw_rate_cmd = msg->yaw_rate;
      double throttle_cmd;

      if (idle_state_before_mission_){
        throttle_cmd = 5.0;
      } else {
        throttle_cmd = msg->thrust.z;
      }
 
    sensor_msgs::Joy Joy_output_msg;
    Joy_output_msg.header.stamp=ros::Time::now();    
    Joy_output_msg.axes.push_back(roll_cmd);
    Joy_output_msg.axes.push_back(pitch_cmd);
    Joy_output_msg.axes.push_back(throttle_cmd); //use vel command instead
    Joy_output_msg.axes.push_back(yaw_rate_cmd);
    Joy_output_msg.axes.push_back(flag);
    ctrlVelYawRatePub.publish(Joy_output_msg);         
  }

}

void rcCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
   

    //bool current_control_status_change=false;
    // bool previous_control_status_change=false;
    input_rc = *joy;
    Joy_stick_input_in_twist.angular.z = 1 * joy->axes[2];
    Joy_stick_input_in_twist.linear.y = 1 * joy->axes[0];
    Joy_stick_input_in_twist.linear.x = 1 * joy->axes[1];
    Joy_stick_input_in_twist.linear.z = 1 * joy->axes[3];
    //for m100: input_rc.axes[4]   F: -8000  A: 0 P: 8000
    //for m100: input_rc.axes[5]   switch down: -4545  A: 0 P: -10000
    if (sim_type_=="vins_dji"){
        if(input_rc.axes[5] <-8000){
            Software_bypass = false;
        } else {
            Software_bypass = true;
        }   
    } else if (sim_type_=="vinsfusion_dji_mini" || sim_type_=="vicon_dji_mini"){ //using futaba RC
        if(input_rc.axes[4] >0.0){
            Software_bypass = false;
        } else {
            Software_bypass = true;
        }          
    }
}

void heightCallback(const std_msgs::Float32::ConstPtr &msg)
{
    height_=msg->data;
}
double expmapping(double input)
{
    double output = 0;

    if(input >= 0)
        output = pow((double)input, (double)(pwrfactor));
    else
        output = -pow(-(double)input, (double)(pwrfactor));



    return output;
}

double scaling(double input, double in_low, double in_high, double out_low, double out_high)
{
    return (out_low+(input-in_low)/(in_high-in_low)*(out_high-out_low));
}


void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    idle_state_before_mission_=false;
    traj_update_time=ros::Time::now();
    traj_true_pub.publish(msg);
    target_pose_=odom_pose_;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "ubplanner_bridge");

    child_name = uav_frame_name;
    ros::Time::init();
    ros::NodeHandle node;

    if (!node.getParam("map_orig_lat", map_orig_lat_)||
        !node.getParam("map_orig_lon", map_orig_lon_)||
        !node.getParam("global_gps_ref", use_gps_)||
        !node.getParam("sim_type", sim_type_)||
        !node.getParam("manual_enable", manual_enable_)||
        !node.getParam("manual_throttle_low",manual_throttle_low_)||
        !node.getParam("manual_throttle_high",manual_throttle_high_)||
        !node.getParam("enable_thrust_control",thrust_control_)){
        std::cout<<"not getting param!";
        exit(-1);
    } else if (sim_type_!="rotors" && sim_type_!="dji" && sim_type_!="vins_dji" && sim_type_!="vins_st" && 
    sim_type_!="sim_st" && sim_type_!="vinsfusion_dji_mini" && sim_type_!="vicon_dji_mini"){
    ROS_WARN("not good, don't know in simulation or real flight");
    exit(-1);
    }
    odometry_starting_time = ros::Time::now();
    //image_transport::ImageTransport it(node);
    //image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
    //ros::Subscriber setpoint_sub = node.subscribe("/dji_sdk/flight_control_setpoint_generic", 1, &setpoint_Callback);
    setpoint_pub = node.advertise<sensor_msgs::Joy>("/st_sdk/flight_control_setpoint_generic", 1);
    ros::Subscriber IMU_sub = node.subscribe("/imu/imu", 1, &imuCallback);

    ros::Subscriber Visual_odometry_sub = node.subscribe("/vins_estimator/odometry", 1, &odometry_Callback);
    //ros::Subscriber cam_pose_sub = node.subscribe("/vins_estimator/camera_pose", 1, &Cam_pose_Callback);
    //ros::Subscriber tracked_feature_sub = node.subscribe("/feature_tracker/feature", 1, &tracked_feature_Callback);





    ros::Subscriber fcc_orientation_sub = node.subscribe("/dji_sdk/attitude", 1, &fcc_orientation_Callback);
    ros::Subscriber fcc_gps_sub;

    if (use_gps_){
        fcc_gps_sub = node.subscribe("/dji_sdk/gps_position", 1, &fcc_gps_Callback); 
    } else {
        fcc_gps_sub = node.subscribe("/gps/fix", 1, &fcc_gps_Callback);
    }
    ros::Subscriber origin_gps_sub = node.subscribe("/gps/fix", 1, &origin_gps_Callback); //for current use

    ros::Subscriber path_in_ubplanner_sub = node.subscribe("/urbanplanner/norminalFlightPath", 1, &path_in_ubplanner_Callback);
    ros::Subscriber st_vel_sub = node.subscribe("/dji_sdk/velocity", 1, &st_vel_cb);
    ros::Subscriber clickedpoint_sub = node.subscribe<geometry_msgs::PointStamped>("/clicked_point", 
                                        10, fix_transform_cb);



    ros::Publisher VIpose3d_var_pub = node.advertise<std_msgs::Float64>("/visual_odometry_varance", 1);
    //ros::Publisher VIpose3d_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/visual_odometry", 1);
    ros::Publisher imu_status = node.advertise<std_msgs::Float64>("/status_imu", 1);
    ros::Publisher img_status = node.advertise<std_msgs::Float64>("/status_camera", 1);
    ros::Publisher odo_status = node.advertise<std_msgs::Float64>("/status_odometry", 1);
    ros::Publisher current_tracked_feature_number = node.advertise<std_msgs::Int8>("/features_num", 1);

    NTU_internal_path_pub= node.advertise<nav_msgs::Path>("/NTU_internal/path_in_local_ref", 1);
    NTU_internal_odom_pub= node.advertise<nav_msgs::Odometry>("/NTU_internal/drone_feedback", 1);


    ros::Subscriber command_roll_pitch_yawrate_thrust_sub_ = node.subscribe("/firefly/command/roll_pitch_yawrate_thrust", 1,  //mq
                                                         &commandRollPitchYawrateThrustCallback);
    ros::Subscriber rc_sub_                 = node.subscribe<sensor_msgs::Joy>("/dji_sdk/rc", 1, &rcCallback);
    ros::Subscriber altitude_sub_           = node.subscribe<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 1, &heightCallback);

    //ros::Subscriber flightStatusSub         = node.subscribe("dji_sdk/flight_status", 1, &flight_status_callback);

    ctrlVelYawRatePub       = node.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);  // new dji way
    sdk_ctrl_authority_service              = node.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");


    ros::Subscriber trajectory_sub = node.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                     "/firefly/command/trajectory", 10, trajectory_cb);

    traj_true_pub=node.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory_true", 10);




    current_GPS_feedback_health.data = 1;
    current_flight_mode.data = 99;
    previous_flight_mode.data = 99;
    //ros::spin();
    ros::Rate loop_rate(30);

    tf::TransformListener lr(ros::Duration(1000));
    traj_update_time=ros::Time::now();
    pilot_input_time_=ros::Time::now();
    tran = &lr;
    current_GPS_feedback.longitude =initial_GPS.longitude;
    current_GPS_feedback.latitude =initial_GPS.latitude;
    while (ros::ok())
    {   
        feedback.header.stamp=ros::Time::now();
        feedback.header.frame_id=global_frame_name;
        int _rand=std::rand();
        feedback.pose.pose.position.x = (current_GPS_feedback.longitude -initial_GPS.longitude)*1.1131/0.00001
                                        +_rand%5*0.001;
                                      //(current_GPS_feedback.longitude -map_orig_lon_)*1.1131/0.00001;
        feedback.pose.pose.position.y = (current_GPS_feedback.latitude -initial_GPS.latitude)*1.1131/0.00001
                                        +_rand%5*0.001;
                                      //(current_GPS_feedback.latitude -map_orig_lat_)*1.1131/0.00001;
        feedback.pose.pose.position.z = 0.0f;//current_GPS_feedback.altitude; //-  initial_GPS.altitude;
      //  feedback.pose.pose.orientation.z = orientation_yaw - 3.1415926/2;

        feedback.pose.pose.orientation.x = compass_quaternion.quaternion.x;
        feedback.pose.pose.orientation.y = compass_quaternion.quaternion.y;
        feedback.pose.pose.orientation.z = compass_quaternion.quaternion.z;
        feedback.pose.pose.orientation.w = compass_quaternion.quaternion.w;

        // from NEU(seriously?) to ENU frame
        feedback.twist.twist.linear.x = 0.0;//st_vel.x;
        feedback.twist.twist.linear.y = 0;//st_vel.y;
        feedback.twist.twist.linear.z = 0;//st_vel.z;

        NTU_internal_odom_pub.publish(feedback);

        // std::cout<<"altitude offset is : "<<altitude_offset_<<"\n";
        // std::cout<<"height is : "<<height_<<"\n";
        // std::cout<<"odom z is : "<<odom_pose_.pose.pose.position.z<<"\n";
        if(transform_fixed)
        {
            static tf::TransformBroadcaster br;

            br.sendTransform(tf::StampedTransform(fixed_transform_world_uav, 
                ros::Time::now(), global_frame_name, uav_frame_name));
        }

        if(!Software_bypass)
        {
            if(Software_bypass != previousSoftware_bypass)
            {
                sdkAuthority.request.control_enable = 0;
                sdk_ctrl_authority_service.call(sdkAuthority);

                std::cout << "drop_control, stop virtual stick and mission" << std::endl;
            }
        }
        else
        {
            if(Software_bypass != previousSoftware_bypass)
            {
                sdkAuthority.request.control_enable = 1;
                sdk_ctrl_authority_service.call(sdkAuthority);
                std::cout << "Software Control Enabled" << std::endl;
                target_pose_=odom_pose_;


                if (!transform_fixed||!got_path_){
                    ROS_WARN("not getting transform yet! cannot generate waypoint in local frame");
                } else {
                    got_path_=false;
                    nav_msgs::Path current_path_local_ref; 
                    current_path_local_ref.header.stamp=ros::Time::now();
                    current_path_local_ref.header.frame_id=uav_frame_name;
                    for( int i =0; i< current_path_map_frame_.poses.size(); i++){
                        geometry_msgs::PoseStamped path_pose_local_ref;
                        path_pose_local_ref.header.frame_id=uav_frame_name;
                        tf2::doTransform(current_path_map_frame_.poses[i], path_pose_local_ref, fixed_transform_world_uav_tf2);
                        current_path_local_ref.poses.push_back(path_pose_local_ref);

                    }
                    // std::cout << "\n\ncurrent_path_map frame\n" << current_path_map_frame_ << std::endl;
                    // std::cout << "\ncurrent_path_local frame\n" << current_path_local_ref << std::endl;

                    NTU_internal_path_pub.publish(current_path_local_ref);

                }
            }
        }
        previousSoftware_bypass = Software_bypass;

        if (Software_bypass && manual_enable_){ //enable thrust+roll pitch yaw control
            there_is_pilot_input_=true;
            pilot_input_time_=ros::Time::now();
            //std::cout << (Joy_stick_input_in_twist.linear.x) << " " ;
            //std::cout << (prev_Joy_stick_input_in_twist.linear.x) << std::endl;
            geometry_msgs::Twist Position_command_out;

            double roll = Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.linear.y);
            double pitch = Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.linear.x);
            double yaw = Yaw_Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.angular.z);
            double throttle = scaling((double) Joy_stick_input_in_twist.linear.z, -1.0, 1.0, 
                                        manual_throttle_low_, manual_throttle_high_); //mq
            Position_command_out.linear.y = roll;
            Position_command_out.linear.x = pitch;
            Position_command_out.angular.z = -yaw;
            Position_command_out.linear.z = throttle;

            sensor_msgs::Joy Joy_output_msg;
            Joy_output_msg.header.stamp=ros::Time::now();
            Joy_output_msg.axes.push_back(Position_command_out.linear.y);
            Joy_output_msg.axes.push_back(Position_command_out.linear.x);
            Joy_output_msg.axes.push_back(Position_command_out.linear.z);
            Joy_output_msg.axes.push_back(Position_command_out.angular.z);

            uint8_t _flag = (DJISDK::VERTICAL_THRUST |
                            DJISDK::HORIZONTAL_ANGLE |
                            DJISDK::YAW_RATE |
                            DJISDK::HORIZONTAL_BODY     ///|
                            // DJISDK::STABLE_ENABLE
                           );
            Joy_output_msg.axes.push_back(_flag);

            ctrlVelYawRatePub.publish(Joy_output_msg);   //nonlinear output for control

        }else if (Software_bypass && (ros::Time::now()-  traj_update_time).toSec()>0.5){
            if(Joy_stick_input_in_twist.linear.y != 0 || 
                Joy_stick_input_in_twist.linear.x != 0 || 
                Joy_stick_input_in_twist.angular.z != 0 || 
                Joy_stick_input_in_twist.linear.z != 0){ //if there is pilot rc input

                there_is_pilot_input_=true;
                pilot_input_time_=ros::Time::now();
                //std::cout << (Joy_stick_input_in_twist.linear.x) << " " ;
                //std::cout << (prev_Joy_stick_input_in_twist.linear.x) << std::endl;
                geometry_msgs::Twist Position_command_out;

                double roll = Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.linear.y);
                double pitch = Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.linear.x);
                double yaw = Yaw_Manual_degree_limit * expmapping((double) Joy_stick_input_in_twist.angular.z);
                double throttle = expmapping((double) Joy_stick_input_in_twist.linear.z);
                if (throttle > throttle_out_limit)
                    throttle = throttle_out_limit;
                if (throttle < -throttle_out_limit)
                    throttle = -throttle_out_limit;
                throttle = scaling((double) Joy_stick_input_in_twist.linear.z, -1.0, 1.0, -2.0, 2.0); //mq
                Position_command_out.linear.y = roll;
                Position_command_out.linear.x = pitch;
                Position_command_out.angular.z = -yaw;
                Position_command_out.linear.z = throttle;

                sensor_msgs::Joy Joy_output_msg;
                Joy_output_msg.header.stamp=ros::Time::now();
                Joy_output_msg.axes.push_back(Position_command_out.linear.y);
                Joy_output_msg.axes.push_back(Position_command_out.linear.x);
                Joy_output_msg.axes.push_back(Position_command_out.linear.z);
                Joy_output_msg.axes.push_back(Position_command_out.angular.z);

                uint8_t _flag = (DJISDK::VERTICAL_VELOCITY |
                                DJISDK::HORIZONTAL_ANGLE |
                                DJISDK::YAW_RATE |
                                DJISDK::HORIZONTAL_BODY     ///|
                                // DJISDK::STABLE_ENABLE
                               );
                Joy_output_msg.axes.push_back(_flag);

                ctrlVelYawRatePub.publish(Joy_output_msg);   //nonlinear output for control
                target_pose_=odom_pose_;
            } else if ((ros::Time::now()-  pilot_input_time_).toSec()<0.5){ //no control, let drone come to rest
                there_is_pilot_input_=false;
                float _vx, _vy, _vz;
                _vx=currentdata.twist.twist.linear.x;
                _vy=currentdata.twist.twist.linear.y;
                _vz=currentdata.twist.twist.linear.z;
                float odom_v_square=_vx*_vx+_vy*_vy+_vz*_vz;
                float _speed=sqrt(odom_v_square);
                target_pose_=odom_pose_;
                target_pose_.pose.pose.position.x+=odom_v_square/4*_vx/_speed;
                target_pose_.pose.pose.position.y+=odom_v_square/4*_vy/_speed;
                target_pose_.pose.pose.position.z+=odom_v_square/4*_vz/_speed;

                sensor_msgs::Joy Joy_output_msg;
                Joy_output_msg.header.stamp=ros::Time::now();
                Joy_output_msg.axes.push_back(0.0f);
                Joy_output_msg.axes.push_back(0.0f);
                Joy_output_msg.axes.push_back(0.0f);
                Joy_output_msg.axes.push_back(0.0f);

                uint8_t _flag = (DJISDK::VERTICAL_VELOCITY |
                                DJISDK::HORIZONTAL_ANGLE |
                                DJISDK::YAW_RATE |
                                DJISDK::HORIZONTAL_BODY     ///|
                                // DJISDK::STABLE_ENABLE
                               );
                Joy_output_msg.axes.push_back(_flag);
            } else { //no control. send current odom as target pose
                trajectory_msgs::MultiDOFJointTrajectory traj_msg;
                traj_msg.header.stamp=ros::Time::now();
                traj_msg.header.frame_id=uav_frame_name;
                trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
                geometry_msgs::Transform trans_msg;
                trans_msg.translation.x=target_pose_.pose.pose.position.x;
                trans_msg.translation.y=target_pose_.pose.pose.position.y;
                trans_msg.translation.z=target_pose_.pose.pose.position.z;

                tf::Quaternion q_imu;
                tf::Quaternion q_vins(
                target_pose_.pose.pose.orientation.x,
                target_pose_.pose.pose.orientation.y,
                target_pose_.pose.pose.orientation.z,
                target_pose_.pose.pose.orientation.w);
                q_imu.setRPY(-3.1415927,0,0);
                tf::Quaternion _q_uav=q_vins*q_imu;
                //trans_msg.rotation = target_pose_.pose.pose.orientation;
                trans_msg.rotation.x=_q_uav.x();
                trans_msg.rotation.y=_q_uav.y();
                trans_msg.rotation.z=_q_uav.z();
                trans_msg.rotation.w=_q_uav.w();

                point_msg.transforms.push_back(trans_msg);
                geometry_msgs::Twist _vel_msg, _acc_msg;
                _vel_msg.linear.x=0;
                _vel_msg.linear.y=0;
                _vel_msg.linear.z=0;
                point_msg.velocities.push_back(_vel_msg);
                _acc_msg.linear.x=0;
                _acc_msg.linear.y=0;
                _acc_msg.linear.z=0;
                point_msg.accelerations.push_back(_acc_msg);
                traj_msg.points.push_back(point_msg);
                traj_true_pub.publish(traj_msg);
            }
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
};

