#ifndef _TF_LISTENER_HPP_
#define _TF_LISTENER_HPP_

#include <tf/transform_listener.h>
#include "vector2d.hpp"

namespace utils
{

class TfListener
{
public:
	TfListener(TfListener const&) = delete;
	void operator=(TfListener const&) = delete;
	~TfListener() {}

	static TfListener& getInstance()
   	{
      		static TfListener singleton;
      		return singleton;
	}

	#define TF TfListener::getInstance()

	bool transformPose(double& x, double& y, double& theta, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
		tf::Stamped<tf::Pose> pose,tfPose;
		pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,theta), tf::Vector3(x,y,0)));
		pose.frame_id_ = sourceFrameId;
		pose.stamp_ = ros::Time(0);
		try
		{
			tf_listener.transformPose(targetFrameId, pose, tfPose);
		} catch(std::exception &e) {
			ROS_ERROR("%s",e.what());
			return false;
		}
		x = tfPose.getOrigin().getX();
		y = tfPose.getOrigin().getY();
		tf::Matrix3x3 m(tfPose.getRotation());
		double roll,pitch;
		m.getRPY(roll, pitch, theta);
		return true;
	}

	bool transformPoint(double& x, double& y, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
		tf::Stamped<tf::Pose> pose,tfPose;
		pose.setData(tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(x,y,0)));
		pose.frame_id_ = sourceFrameId;
		pose.stamp_ = ros::Time(0);
		try
		{
			tf_listener.transformPose(targetFrameId, pose, tfPose);
		} catch(std::exception &e) {
			ROS_ERROR("%s",e.what());
			return false;
		}
		x = tfPose.getOrigin().getX();
		y = tfPose.getOrigin().getY();
		return true;
	}
	bool transformPoint(utils::Vector2d& point, const std::string& sourceFrameId, const std::string& targetFrameId) const
	{
		double x = point.getX();
		double y = point.getY();
		if (transformPoint(x,y,sourceFrameId,targetFrameId)) {
			point.set(x,y);
			return true;
		}
		return false;
	}


private:
	TfListener()  {}
	tf::TransformListener tf_listener;
};

}

#endif

