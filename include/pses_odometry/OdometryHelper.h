#ifndef OdometryHelper_H
#define OdometryHelper_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_odometry/ForwardKinematics.h>
#include <sensor_msgs/Imu.h>

#define WHEEL_RADIUS 0.032
#define RAD_PER_TICK 0.7853981634 // 2*PI/8
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS
#define STANDARD_GRAVITY 9.80665 // m/s^2  

class OdometryHelper{
public:
	OdometryHelper() {
		yaw = 0.0;
		pitch = 0.0;
		roll = 0.0;
		dt = 0.0;
		drivenDistance = 0.0;
		speed = 0.0;
		drivingDirection = 0;
        prevDirection = 0;
        wxOffset = 0.0;
        wyOffset = 0.0;
        wzOffset = 0.0;
        axOffset = 0.0;
        ayOffset = 0.0;
        azOffset = 0.0;
        dataCount = 0;
        imuCalibrated = false;
		oldTimeStamp = ros::Time::now();
		odometric.setK(0.25); // set distance between front axis and back axis (in meters)
	}

  inline void updateSensorData(const sensor_msgs::Imu& imuData, const double hallDt) {
    calcDt(imuData.header.stamp, oldTimeStamp);
		if(imuCalibrated) {
      this->imuData.angular_velocity.x = imuData.angular_velocity.x - wxOffset;
      this->imuData.angular_velocity.y = imuData.angular_velocity.y - wyOffset;
      this->imuData.angular_velocity.z = imuData.angular_velocity.z - wzOffset;
      this->imuData.linear_acceleration.x = imuData.linear_acceleration.x - axOffset;
      this->imuData.linear_acceleration.y = imuData.linear_acceleration.y - ayOffset;
      this->imuData.linear_acceleration.z = imuData.linear_acceleration.z - azOffset;
      yaw = integrateEulerAngles(yaw, this->imuData.angular_velocity.z, dt);
      pitch = integrateEulerAngles(pitch, this->imuData.angular_velocity.y, dt);
      roll = integrateEulerAngles(roll, this->imuData.angular_velocity.x, dt);
		}
    else calibrateIMU();
    this->hallDt = hallDt;
    oldTimeStamp = imuData.header.stamp;
		calcSpeed();
		calcDeltaDistance();
		calcDrivenDistance();
		calcPosition();
	}


  inline void updateMotorLevel(const int motorLevel) {
    this->motorLevel = motorLevel;
        prevDirection = drivingDirection;
        if(motorLevel < 0) drivingDirection = -1;
    else if(motorLevel > 0) drivingDirection = 1;
		else drivingDirection = 0;
	}

	inline double getYaw() {
		return yaw;
	}
	inline double getRoll() {
		return roll;
	}
	inline double getPitch() {
		return pitch;
	}
	inline float getSpeed() {
		return speed;
	}
	inline void getQuaternion(geometry_msgs::Quaternion& quat) {
		quat = tf::createQuaternionMsgFromYaw(yaw);
	}
	inline float getDrivenDistance() {
		return drivenDistance;
	}
	inline geometry_msgs::Point getPosition() {
		return position;
	}
	inline bool isImuCalibrated() {
		return imuCalibrated;
	}
private:
	double yaw;
	double roll;
	double pitch;
	double dt;
	float speed;
	float drivenDistance;
	float deltaDistance;
    int prevDirection;
	int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards
	
	//IMU offsets
	double wxOffset;
	double wyOffset;
	double wzOffset;
	double axOffset;
	double ayOffset;
	double azOffset;

	uint32_t dataCount;
	bool imuCalibrated;
	ros::Time oldTimeStamp;
	geometry_msgs::Point position;
  ForwardKinematics odometric;
  int motorLevel;
  double hallDt;
  sensor_msgs::Imu imuData;
	inline void calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp) {
		dt = (currentTimeStamp - oldTimeStamp).toSec();
	}

	inline void calibrateIMU() {
    if(dataCount < 500) {
			dataCount++;
      wxOffset += imuData.angular_velocity.x;
      wyOffset += imuData.angular_velocity.y;
      wzOffset += imuData.angular_velocity.z;
      axOffset += imuData.linear_acceleration.x;
      ayOffset += imuData.linear_acceleration.x;
      azOffset += (imuData.linear_acceleration.z - STANDARD_GRAVITY);
		}
		else if(dataCount == 500 && !imuCalibrated) {
			wxOffset /= dataCount;
			wyOffset /= dataCount;
			wzOffset /= dataCount;
			axOffset /= dataCount;
			ayOffset /= dataCount;
			azOffset /= dataCount;
			imuCalibrated = true;
		}
	}

	inline void calcRPY(){
	}
	inline double integrateEulerAngles(double angle, double dAngle, double dT){
		double result = angle + dAngle*dT;
			if(result>M_PI){
				return -2*M_PI+result;
			}else if(result<-M_PI){
				return 2*M_PI+result;
			}else{
				return result;
			}
	}

	inline void calcSpeed() {
        if(!std::isnan(hallDt)){
            speed =  drivingDirection * DRIVEN_DISTANCE_PER_TICK / hallDt;
        }else{
            if(prevDirection!=drivingDirection) speed = 0;
            else if(drivingDirection == 0) speed =  0;
        }
	}


	inline void calcDeltaDistance() {
    deltaDistance = std::isnan(hallDt)?0.0 : drivingDirection*DRIVEN_DISTANCE_PER_TICK;
	}

	inline void calcDrivenDistance() {
		drivenDistance = drivenDistance + deltaDistance;
	}
	inline void calcPosition() {
		std::vector<double> pos = odometric.getUpdateWithGyro(yaw, deltaDistance);
		position.x = pos.at(1);
		position.y = -pos.at(0);
		position.z = 0.0;
	}
};

#endif
