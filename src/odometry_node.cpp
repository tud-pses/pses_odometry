#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pses_odometry/OdometryHelper.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

/*
void buildOdometryTransformation(geometry_msgs::TransformStamped& odomTransform,
                                 const pses_basis::SensorData& sensorData,
                                 OdometryHelper& odomHelper,
                                 geometry_msgs::Quaternion& odomQuaternion)
{

  odomTransform.header.stamp = sensorData.header.stamp;
  odomTransform.header.frame_id = "odom";
  odomTransform.child_frame_id = "base_footprint";
  geometry_msgs::Point position = odomHelper.getPosition();
  odomTransform.transform.translation.x = position.x;
  odomTransform.transform.translation.y = position.y;
  odomTransform.transform.translation.z = 0.0;
  odomTransform.transform.rotation = odomQuaternion;
}
*/
/*
void buildOdometryMessage(nav_msgs::Odometry& odom,
                          const pses_basis::SensorData& sensorData,
                          OdometryHelper& odomHelper,
                          geometry_msgs::Quaternion& odomQuaternion)
{

  odom.header.stamp = sensorData.header.stamp;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position = odomHelper.getPosition();
  odom.pose.pose.orientation = odomQuaternion;

  // set the velocity
  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x =
      odomHelper.getSpeed() * std::cos(sensorData.angular_velocity_z);
  odom.twist.twist.linear.y =
      odomHelper.getSpeed() * std::sin(sensorData.angular_velocity_z);
  odom.twist.twist.angular.z = sensorData.angular_velocity_z;
}
*/
/*
void buildInfoMessage(pses_basis::CarInfo& info,
                      const pses_basis::SensorData& sensorData,
                      OdometryHelper& odomHelper)
{
  // configure header
  info.header.stamp = sensorData.header.stamp;
  info.header.frame_id = sensorData.header.frame_id;
  info.header.seq++;
  info.header.car_id = sensorData.header.car_id;
  // set rpy-Angles
  info.roll = odomHelper.getRoll();
  info.pitch = odomHelper.getPitch();
  info.yaw = odomHelper.getYaw();
  // set driven distance
  info.driven_distance = odomHelper.getDrivenDistance();
  // set speed
  info.speed = odomHelper.getSpeed();
}

void commandCallback(const pses_basis::Command::ConstPtr& cmd,
                     OdometryHelper* odomHelper)
{
  // update the command information for later odometric calculations
  odomHelper->updateCommand(*cmd);
}
*/
/*
void dataCallback(const pses_basis::SensorData::ConstPtr& sensorData,
                  tf::TransformBroadcaster* odomBroadcaster,
                  ros::Publisher* odom_pub, OdometryHelper* odomHelper)
{

  pses_basis::SensorData sensorDataCopy = *sensorData;
  // update sensor data for later odometric calculations
  odomHelper->updateSensorData(sensorDataCopy);
  // objects to store the odometry and its transform
  geometry_msgs::TransformStamped odomTransform;
  nav_msgs::Odometry odom;
  pses_basis::CarInfo info;
  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odomQuaternion;
  odomHelper->getQuaternion(odomQuaternion);
  // build odom transform for tf and the odometry message
  buildOdometryTransformation(odomTransform, sensorDataCopy, *odomHelper,
                              odomQuaternion);
  buildOdometryMessage(odom, sensorDataCopy, *odomHelper, odomQuaternion);
  buildInfoMessage(info, sensorDataCopy, *odomHelper);
  // send the transform
  odomBroadcaster->sendTransform(odomTransform);
  // publish the odometry message
  odom_pub->publish(odom);
  carInfo_pub->publish(info);
}
*/

void imuCallback(sensor_msgs::Imu::ConstPtr msg, sensor_msgs::Imu* out){
  *out = *msg;
}

void motorCallback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* out){
  *out = *msg;
}

void hallCntCallback(std_msgs::UInt8::ConstPtr msg, std_msgs::UInt8* out){
  *out = *msg;
}

void hallDtCallback(std_msgs::Float64::ConstPtr msg, std_msgs::Float64* out){
  *out = *msg;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "odometry");

  ros::NodeHandle nh;
  // object needed to send odometric information to the navigational stack
  tf::TransformBroadcaster odomBroadcaster;
  geometry_msgs::TransformStamped odomTransform;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odomQuaternion;
  // object neeeded to store the current commands
  sensor_msgs::Imu imu;
  std_msgs::Int16 motor;
  std_msgs::UInt8 hallCnt;
  std_msgs::Float64 hallDt;
  // objects needed for odometric calculations
  OdometryHelper odomHelper;
  // Publishes the results of the odometry calculations to other ros nodes
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("/uc_bridge/imu", 10, boost::bind(imuCallback,_1,&imu));
  ros::Subscriber motorSub = nh.subscribe<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1, boost::bind(motorCallback,_1,&motor));
  ros::Subscriber hallCntSub = nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hallCntCallback,_1,&hallCnt));
  ros::Subscriber hallDtSub = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt", 10, boost::bind(hallDtCallback,_1,&hallDt));

  // Loop starts here:
  ros::Rate loop_rate(150);
  while (ros::ok())
  {
    //odomHelper->updateSensorData(sensorDataCopy);


    odomPub.publish(odom);
    odomBroadcaster.sendTransform(odomTransform);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
