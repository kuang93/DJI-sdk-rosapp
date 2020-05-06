/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk/dji_sdk.h"
#include <Eigen/Geometry>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <keyboard/Key.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#define KEYCODE_T 0x74   // takeoff
#define KEYCODE_H 0x68   // land
#define KEYCODE_UP 273   // up
#define KEYCODE_DOWN 274   // down
#define KEYCODE_W 0x77   // forward
#define KEYCODE_A 0x61   // left
#define KEYCODE_S 0x73   // backward
#define KEYCODE_D 0x64   // right
#define KEYCODE_Q 0x71   // turn left
#define KEYCODE_E 0x65   // turn right


#define C_PI (double)3.141592653589793

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

class AircraftControl
{
  public:    
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;

    //ros::Publisher ctrlPosYawPub;
    ros::Publisher ctrlVelYawRatePub;
    ros::Publisher ctrlBrakePub;

    ros::Subscriber attitudeSub;
    ros::Subscriber gpsSub ;
    ros::Subscriber flightStatusSub;
    ros::Subscriber displayModeSub ;
    ros::Subscriber localPosition ;
    ros::Subscriber keyDownSub ;
    ros::Subscriber keyUpSub ;

    ros::Timer timer;

    // global variables for subscribed topics
    uint8_t flight_status = 255;
    uint8_t display_mode  = 255;
    sensor_msgs::NavSatFix current_gps;
    geometry_msgs::Quaternion current_atti;
    geometry_msgs::Point current_local_pos;
    
    ros::NodeHandle nh;

    unsigned int keydown, keyup;
    sensor_msgs::Joy controlVelYawRate;
    double linearVelocity;
    bool takeoff_result;
    bool pub_vel;
    
  public:
    AircraftControl(): keydown(0), keyup(0),takeoff_result(false),pub_vel(false)
    {

      ros::NodeHandle pnh("~");
      pnh.param("linear_velocity", linearVelocity, 0.5);
      // Subscribe to messages from dji_sdk_node
      attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &AircraftControl::attitude_callback, this);
      gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &AircraftControl::gps_callback, this);
      flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &AircraftControl::flight_status_callback, this);
      displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &AircraftControl::display_mode_callback, this);
      localPosition = nh.subscribe("dji_sdk/local_position", 10, &AircraftControl::local_position_callback, this);
      
      // Subscribe to messages from ros_keyboard
      keyDownSub = nh.subscribe("/keyboard/keydown", 10, &AircraftControl::key_down_callback, this);
      keyUpSub = nh.subscribe("/keyboard/keyup", 10, &AircraftControl::key_up_callback, this);

      // Publish the control signal
      //ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
      ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>
          ("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
      controlVelYawRate.axes.push_back(0); // x axes;
      controlVelYawRate.axes.push_back(0); // y axes;
      controlVelYawRate.axes.push_back(0); // z axes;
      controlVelYawRate.axes.push_back(0); // yaw rate;

  
      // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
      // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
      // properly in function Mission::step()
      // ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
      // Basic services
      sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
      drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
      query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
      set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
      
      
      bool obtain_control_result = obtain_control();
      
      if (!set_local_position()) // We need this for height
      {
        ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
        exit(0);
      }

      timer = nh.createTimer(ros::Duration(1.0 / 10), &AircraftControl::topic_pub, this);
    }
    
    bool takeoff_land(int task);
    bool obtain_control();
    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
    void key_down_callback(const keyboard::Key& msg);
    void key_up_callback(const keyboard::Key& msg);
    bool monitoredTakeoff();
    bool monitoredLand();
    bool set_local_position();
    void topic_pub(const ros::TimerEvent& event); 

};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_control");
  AircraftControl ac;
  
  // ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 10), &AircraftControl::topic_pub, &ac);

  ros::spin();
  
  return 0;
}


//void controlAircraft(const ros::TimerEvent& event)
//{}

void AircraftControl::topic_pub(const ros::TimerEvent& event)
{
  // if(pub_vel)
    {
      ctrlVelYawRatePub.publish(controlVelYawRate);
    }
}


bool AircraftControl::takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool AircraftControl::obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

void AircraftControl::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void AircraftControl::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void AircraftControl::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
}

void AircraftControl::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void AircraftControl::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void AircraftControl::key_down_callback(const keyboard::Key& msg)
{
  keydown = msg.code;

  Eigen::Quaterniond current_q(current_atti.w, current_atti.x, current_atti.y, current_atti.z);
  // double currentYaw = current_q.toRotationMatrix().eulerAngles(0,1,2)[2];
  // ROS_INFO("currentYaw 2 =%f",currentYaw*rad2deg);
  // std::cout<<current_q.matrix().eulerAngles(0,1,2)<<std::endl;

  Eigen::Vector3d velocity_body;
  Eigen::Vector3d velocity_ENU;

  switch(keydown){
    case KEYCODE_T:
      ROS_INFO("M210 taking off!");
      takeoff_result = monitoredTakeoff();
      if(takeoff_result) ROS_INFO("takeoff sucess!");
      pub_vel = true;
      break;
    case KEYCODE_H:
      ROS_INFO("M210 landding!");
      pub_vel = false;
      takeoff_result = !monitoredLand();
      if(!takeoff_result) ROS_INFO("land sucess!");
      break;
    case KEYCODE_UP:
      ROS_INFO("UP! velocity=%dm/s",linearVelocity);
      controlVelYawRate.axes[2] = linearVelocity; // up velocity 
      break;
    case KEYCODE_DOWN:
      ROS_INFO("Down! velocity=%dm/s",linearVelocity);
      controlVelYawRate.axes[2] = -linearVelocity; // down velocity 
      break;
    case KEYCODE_W:
      ROS_INFO("Forward! velocity=%dm/s",linearVelocity);
      velocity_body << linearVelocity,0,0;
      // controlVelYawRate.axes[0] = linearVelocity * std::cos(currentYaw);  // forward velocity 
      // controlVelYawRate.axes[1] = linearVelocity * std::sin(currentYaw);  // forward velocity 
      break;
    case KEYCODE_S:
      ROS_INFO("Backward! velocity=%dm/s",linearVelocity);
      velocity_body << -linearVelocity,0,0;
      // controlVelYawRate.axes[0] = linearVelocity * std::cos(currentYaw + C_PI);  // backward velocity 
      // controlVelYawRate.axes[1] = linearVelocity * std::sin(currentYaw + C_PI);  // backward velocity 
      break;
    case KEYCODE_A:
      ROS_INFO("Left! velocity=%dm/s",linearVelocity);
      velocity_body << 0,linearVelocity,0;
      // controlVelYawRate.axes[0] = linearVelocity * std::cos(currentYaw + C_PI/2.0);  //left velocity
      // controlVelYawRate.axes[1] = linearVelocity * std::cos(currentYaw + C_PI)/2.0;  //left velocity
      break;
    case KEYCODE_D:
      ROS_INFO("Right! velocity=%dm/s",linearVelocity);
      velocity_body << 0,-linearVelocity,0;
      // controlVelYawRate.axes[0] = linearVelocity * std::cos(currentYaw - deg2rad/2.0);  //right velocity
      // controlVelYawRate.axes[1] = linearVelocity * std::cos(currentYaw - deg2rad/2.0);  //right velocity
      break;
    case KEYCODE_Q:
      ROS_INFO("Turn Left! velocity=%ddeg/s",45.0);
      controlVelYawRate.axes[3] = 90/2*deg2rad;  //turn left velocity
      break;
    case KEYCODE_E:
      ROS_INFO("Turn Right! velocity=%ddeg/s",45.0);
      controlVelYawRate.axes[3] = -90/2*deg2rad;  //turn right velocity
      break;
    default:
      ;
  }
  velocity_ENU = current_q.toRotationMatrix() * velocity_body;
  // std::cout<<velocity_ENU<<std::endl;
  controlVelYawRate.axes[0] = velocity_ENU[0];
  controlVelYawRate.axes[1] = velocity_ENU[1];
}

void AircraftControl::key_up_callback(const keyboard::Key& msg)
{
  keyup = msg.code;

  switch(keyup){
    case KEYCODE_UP:
      controlVelYawRate.axes[2] = 0; // up velocity 
      break;
    case KEYCODE_DOWN:
      controlVelYawRate.axes[2] = 0; // down velocity 
      break;
    case KEYCODE_W:
      controlVelYawRate.axes[0] = 0;  // forward velocity 
      controlVelYawRate.axes[1] = 0;  // forward velocity 
      break;
    case KEYCODE_S:
      controlVelYawRate.axes[0] = 0;  // backward velocity 
      controlVelYawRate.axes[1] = 0;  // backward velocity 
      break;
    case KEYCODE_A:
      controlVelYawRate.axes[0] = 0;  //left velocity
      controlVelYawRate.axes[1] = 0;  //left velocity
      break;
    case KEYCODE_D:
      controlVelYawRate.axes[0] = 0;  //right velocity
      controlVelYawRate.axes[1] = 0;  //right velocity
      break;
    case KEYCODE_Q:
      controlVelYawRate.axes[3] = 0;  //turn left velocity
      break;
    case KEYCODE_E:
      controlVelYawRate.axes[3] = 0;  //turn right velocity
      break;
    default:
      ;
  }
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
AircraftControl::monitoredTakeoff()
{
  if(flight_status == DJISDK::FlightStatus::STATUS_IN_AIR )
    {
      ROS_INFO("Drone has already takeoff!");
      return true;
    }

  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

bool
AircraftControl::monitoredLand()
{
  if(flight_status == DJISDK::FlightStatus::STATUS_STOPPED)
    {
      ROS_INFO("Drone has already land!");
      return true;
    }

  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  while (flight_status != DJISDK::FlightStatus::STATUS_STOPPED &&
         ros::Time::now() - start_time < ros::Duration(60) && ros::ok()) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(60)) {
    ROS_ERROR("land failed. drone is in the air.");
    return false;
  }
  
  ROS_INFO("Successful takeoff!");
  ros::spinOnce();

  return true;
}

bool AircraftControl::set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
