#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <vikit/output_helper.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <kalman_filter/ValidationGuard.h>
#include <kalman_filter/adafruit_msg.h>
#include <kalman_filter/KalmanFilter.h>
#include <kalman_filter/KalmanOrientation.h>
#include <kalman_filter/HelperFunc.h>

using namespace std;

// IMU callback function
 void IMUCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin,   // listen "adafruit/imu"  msgin mesasage is Quaternion + Position of IMU
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator,
		  ros::Publisher& kalmanIndicator, 
		  KalmanFilter10& mainFilter, 
		  KalmanOrientation& orientationFilter,
		  double* q, 
		  ValidationGuard& guard,
		  ros::Publisher& publisher)
{
  // Compute rotation matrix
  q[0] = (double)msgin->transform.rotation.w; // q_es came from teensy orientation fusion 
  q[1] = -1 * (double)msgin->transform.rotation.x; 
  q[2] = -1 * (double)msgin->transform.rotation.y; 
  q[3] = (double)msgin->transform.rotation.z;
  // normalization of quaternion 
  //ROS_INFO("Pre normalisation, norm=%e", (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
  //long double recipNorm = 1 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  //q[1] *= recipNorm;
  //q[2] *= recipNorm;
  //q[3] *= recipNorm;
  //q[0] *= recipNorm;
  //ROS_INFO("Post normalisation, norm=%e", (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));

  //cout << "IMU pose msgs rotation:" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " translation " << msgin->transform.translation.x << " " << msgin->transform.translation.y << " " << msgin->transform.translation.z << endl;
  
  // update the imu dalay buffer
  Vector4d q_vec;
  q_vec << q[0], q[1], q[2], q[3];
  mainFilter.q_bf.push_back(q_vec);
  if (mainFilter.q_bf.size() == (mainFilter.delayed_imu_readings_+1)) {// num of delayed measurements + 1  
  //if (mainFilter.q_bf.size() == (4+1)) {// num of delayed measurements + 1  
    // delet old buffer elements
    mainFilter.q_bf.pop_front(); //delete first element "pop_front"
  } 
  
  // convert q to R
  double _2qxqx = 2*q[1]*q[1];  double _2qyqy = 2*q[2]*q[2];  double _2qzqz = 2*q[3]*q[3];
  double _2qxqy = 2*q[1]*q[2];  double _2qxqz = 2*q[1]*q[3];  double _2qxqw = 2*q[1]*q[0];
  double _2qyqz = 2*q[2]*q[3];  double _2qyqw = 2*q[2]*q[0];
  double _2qzqw = 2*q[3]*q[0];
  
  Matrix3d R; // rotation matrix 
  R<< (1 - _2qyqy - _2qzqz), (_2qxqy - _2qzqw),     (_2qxqz + _2qyqw),
      (_2qxqy + _2qzqw),     (1 - _2qxqx - _2qzqz), (_2qyqz - _2qxqw),
      (_2qxqz - _2qyqw),     (_2qyqz + _2qxqw),     (1 - _2qxqx - _2qyqy);

/*-----------
  if(posIndicator.getNumSubscribers() > 0){
//cout << " imu running" << endl;
    // tf visualisation (for debug)
    tf::Transform transform_msg;
    //transform_msg.setOrigin(tf::Vector3(msgin->transform.translation.x/10.0, msgin->transform.translation.y/10.0, msgin->transform.translation.z/10.0));
    transform_msg.setOrigin(tf::Vector3(-1 * msgin->transform.translation.x/20.0, -1 * msgin->transform.translation.y/20.0, msgin->transform.translation.z/20.0));
    tf::Quaternion tf_q; 
    //tf_q.setX(-1 * msgin->transform.rotation.x); tf_q.setY(-1 * msgin->transform.rotation.y); tf_q.setZ(msgin->transform.rotation.z); tf_q.setW(msgin->transform.rotation.w);
    tf_q.setX(orientationFilter.state_q[1]); tf_q.setY(orientationFilter.state_q[2]); tf_q.setZ(orientationFilter.state_q[3]); tf_q.setW(orientationFilter.state_q[0]);
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "filtered_imu_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(posIndicator, "filtered_imu_pos", "imus", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }

*/
//---------------------
  if(posIndicator.getNumSubscribers() > 0){
//cout << " imu running" << endl;
    // tf visualisation (for debug)
    tf::Transform transform_msg;
    //transform_msg.setOrigin(tf::Vector3(msgin->transform.translation.x/10.0, msgin->transform.translation.y/10.0, msgin->transform.translation.z/10.0));
    transform_msg.setOrigin(tf::Vector3(-1 * msgin->transform.translation.x/10.0, -1 * msgin->transform.translation.y/10.0, msgin->transform.translation.z/10.0));
    tf::Quaternion tf_q; 
    tf_q.setX(-1 * msgin->transform.rotation.x); tf_q.setY(-1 * msgin->transform.rotation.y); tf_q.setZ(msgin->transform.rotation.z); tf_q.setW(msgin->transform.rotation.w);
    //tf_q.setX(orientationFilter.state_q[1]); tf_q.setY(orientationFilter.state_q[2]); tf_q.setZ(orientationFilter.state_q[3]); tf_q.setW(orientationFilter.state_q[0]);
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "imu_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(posIndicator, "imu_pos", "imus", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }
//------------------

  if(kalmanIndicator.getNumSubscribers() > 0){
//cout << " imu running" << endl;
    // tf visualisation (for debug)
    tf::Transform transform_msg;
    transform_msg.setOrigin(tf::Vector3(mainFilter.state[0], mainFilter.state[1], mainFilter.state[2]));
    tf::Quaternion tf_q; 
    //tf_q.setX(mainFilter.qVision[1]); tf_q.setY(mainFilter.qVision[2]); tf_q.setZ(mainFilter.qVision[3]); tf_q.setW(mainFilter.qVision[0]);
    tf_q.setX(orientationFilter.state_q[1]); tf_q.setY(orientationFilter.state_q[2]); tf_q.setZ(orientationFilter.state_q[3]); tf_q.setW(orientationFilter.state_q[0]);
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "kalman_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(kalmanIndicator, "kalman_pos", "kms", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }

//-------------------
  // EKF imu measurement update
  Vector3d a; a << msgin->transform.translation.x, msgin->transform.translation.y, msgin->transform.translation.z;  // raw acceleration imu
  a *= 9.8;  // rescale the acceleration to m/ss

  // update objects
  mainFilter.StateUptSen(a, R, msgin->header.stamp.toSec());
  guard.imu_Cb(q);    // NOTE: q ları imu_q ya eşitliyor q ları degiştirecek buyuk ıhtımal
  	//cout << "2" << endl;

  //---------------
  Vector4d q_sensor_imu;
  q_sensor_imu << q[0], q[1], q[2], q[3];

  orientationFilter.fullkalmanfilter(q_sensor_imu);
  cout << "imu callback" << endl;
// ------------------

  // publish measurments of output
  if (publisher.getNumSubscribers() > 0) {
	cout << "1" << endl;
	kalman_filter::adafruit_msg output;
	output.qw     = (float)mainFilter.qVision[0];    // q_es output  
	output.qx     = (float)mainFilter.qVision[1];
	output.qy     = (float)mainFilter.qVision[2];
	output.qz     = (float)mainFilter.qVision[3];
	output.px     = (float)mainFilter.state[0];      //  p_es output
	output.py     = (float)mainFilter.state[1];
	output.pz     = (float)mainFilter.state[2];
	output.vx     = (float)mainFilter.state[3];      //  v_es output
	output.vy     = (float)mainFilter.state[4];
	output.vz     = (float)mainFilter.state[5];
	output.bx     = (float)mainFilter.state[6];       // b_s accelerameter bias
	output.by     = (float)mainFilter.state[7];
	output.bz     = (float)mainFilter.state[8];
	output.lambda = (float)mainFilter.T;   // metric scale lamda > 0, which is defined p'_vs = lamda * p_vs | state[9];//(float)(ros::Time::now().toSec()-946688000.0);//
	output.status = guard.commitOutput();
	publisher.publish(output);
  }

}

// MINE camera exposure Control
void ExposureCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin, 
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator, 
		  KalmanFilter10& mainFilter)
{
  if(mainFilter.reset_exposure == 0 && msgin->header.frame_id == "Exposure") {
    system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure false");  
    mainFilter.reset_exposure = 1;
  } else if (mainFilter.reset_exposure == 1 && msgin->header.frame_id == "Teensy") {
    system("rosrun dynamic_reconfigure dynparam set ueye_cam_nodelet lock_exposure true");
    mainFilter.reset_exposure = 0;
  }	  
}


//____________________________
// Vision callback function
 void VISCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgin,   // Visual pose coming with "svo/pose" topic
		  tf::TransformBroadcaster& posPublisher, 
		  ros::Publisher& posIndicator,
		  ros::Publisher& kalmanIndicator, 
		  KalmanFilter10& mainFilter,
		  KalmanOrientation& orientationFilter)
{ 
  // Copy message
  double cam_trans[4] = {0.0, msgin->pose.pose.position.x,    msgin->pose.pose.position.z,   -1 * msgin->pose.pose.position.y};
  mainFilter.qVision[0] = msgin->pose.pose.orientation.w;
  mainFilter.qVision[1] = msgin->pose.pose.orientation.x;
  mainFilter.qVision[2] = msgin->pose.pose.orientation.z;
  mainFilter.qVision[3] = -1 * msgin->pose.pose.orientation.y;

  //cout << "CAM pose msgs rotation:" << msgin->pose.pose.orientation.w << " " << msgin->pose.pose.orientation.x << " " << msgin->pose.pose.orientation.y << " " << msgin->pose.pose.orientation.z << " translation " << msgin->pose.pose.position.x << " " << msgin->pose.pose.position.y << " " << msgin->pose.pose.position.z << endl;

  // publish quaternion
  //if(ekfPublisher_quaternion.getNumSubscribers() > 0) ekfPublisher_quaternion.publish(msgin->pose.pose.orientation);
  //cout << mainFilter.state[9] << endl;
  // IMU camera position compensation
  //double IMUpos_v[4] = {0.0, 0.0*mainFilter.state[9], 0.0*mainFilter.state[9], 0.06*mainFilter.state[9]}; // IMU position in vision frame {0.0, forward-x, left-y, up-z} in m (z = p*lambda) (from camera optical centre)
  
  double IMUpos_v[4] = {0.0, 0.0*mainFilter.state[9], 0.0*mainFilter.state[9], 0.0*mainFilter.state[9]};
  double imu_trans[4];
  QuatRot(IMUpos_v, mainFilter.qVision, imu_trans); // compute compensation                  ***** ın thıs section define the IMU position relative to camera and find IMU translation 
  imu_trans[1] += cam_trans[1];          // add camera position measurements					   with depends on camera.
  imu_trans[2] += cam_trans[2];
  imu_trans[3] += cam_trans[3];
  
  // estimate vision gravity
  //double g_s[4] = {0.0, 0.0, 0.0, -1.0}; // gravity in sensor frame
  //double g_v[4];
  //VPos2SPose(g_s, mainFilter.qVision, q_delayed, g_v); // here we use VPos2SPose() inversely to convert from sensor frame to vision frame
  //mainFilter.vision_gravity
  
  // Convert to gravity frame
  //Vector4d q_vec = mainFilter.q_bf.front();
  //double q_delayed[4] = {q_vec(0), q_vec(1), q_vec(2), q_vec(3)};
  //double pos_w[4];
  //VPos2SPose(imu_trans, q_delayed, mainFilter.qVision, pos_w); // note to use the delayed q, mainFilter.q_bf.front()

  // ######### EKF vision measurement update #######
  Vector3d p;	p << imu_trans[1], imu_trans[2], imu_trans[3];
  //Vector3d p;	p << pos_w[1], pos_w[2], pos_w[3];
  //std::cout<< ros::Time::now().toSec()-msgin->header.stamp.toSec()  <<std::endl;
  mainFilter.MeasureUptVis(p);

  //---------------
  Vector4d q_sensor_vis;
  q_sensor_vis << msgin->pose.pose.orientation.w, msgin->pose.pose.orientation.x, msgin->pose.pose.orientation.z, -1 * msgin->pose.pose.orientation.y;

  orientationFilter.fullkalmanfilter(q_sensor_vis);
  cout << "vision callback" << endl;
  // --------------------------
  
  // For visualisation
  if(posIndicator.getNumSubscribers() > 0){
    // /tf for visualisation
    tf::Transform transform_msg;
    tf::Quaternion tf_q;

//cout << " i am running" << endl;

    //tf visualisation (for debug) the visualisation should be the position output from vi_ekf fusion with correct orientation.
    transform_msg.setOrigin(tf::Vector3(mainFilter.state[0], mainFilter.state[1], mainFilter.state[2]));
    //transform_msg.setOrigin(tf::Vector3(msgin->pose.pose.position.x, msgin->pose.pose.position.y, msgin->pose.pose.position.z));
    tf_q.setW(msgin->pose.pose.orientation.w);
    tf_q.setX(msgin->pose.pose.orientation.x);
    tf_q.setZ(-1 * msgin->pose.pose.orientation.y);
    tf_q.setY(msgin->pose.pose.orientation.z);

    //cout << "setorigin" << mainFilter.state[0] << " " << mainFilter.state[1] << " " << mainFilter.state[2] << endl;

    // sent /tf message
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "svo_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(posIndicator, "svo_pos", "svos", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }

    //kalman tf
  if(kalmanIndicator.getNumSubscribers() > 0){
//cout << " imu running" << endl;
    // tf visualisation (for debug)
    tf::Transform transform_msg;
    transform_msg.setOrigin(tf::Vector3(mainFilter.state[0], mainFilter.state[1], mainFilter.state[2]));
    tf::Quaternion tf_q; 
    //tf_q.setX(mainFilter.qVision[1]); tf_q.setY(mainFilter.qVision[2]); tf_q.setZ(mainFilter.qVision[3]); tf_q.setW(mainFilter.qVision[0]);
    tf_q.setX(orientationFilter.state_q[1]); tf_q.setY(orientationFilter.state_q[2]); tf_q.setZ(orientationFilter.state_q[3]); tf_q.setW(orientationFilter.state_q[0]);
    transform_msg.setRotation(tf_q);
    posPublisher.sendTransform(tf::StampedTransform(transform_msg, ros::Time::now(), "world", "kalman_pos"));
    // marker
    vk::output_helper::publishHexacopterMarker(kalmanIndicator, "kalman_pos", "kms", ros::Time::now(), 1, 0, 0.3, Vector3d(0.,0.,1.));
  }
}

//______________________________
// Filter reset callback function
void resetCallback(const geometry_msgs::TransformStamped::ConstPtr& msgin,
		   KalmanFilter10& mainFilter,
		   ValidationGuard& guard)
{
	cout << "44" << endl;
  mainFilter.toReset = true;
  mainFilter.resetLeft = mainFilter.reset_loop;
  guard.reset();
}


// _______________________________
// this is called when svo sends reset signal
void SVOresetCallback(const std_msgs::Bool::ConstPtr& msgin, 
		      KalmanFilter10& mainFilter,
		      ros::Publisher& resetPublisher,
		      double* q)
{
  geometry_msgs::TransformStamped msgout;
  msgout.transform.rotation.w = q[0]; msgout.transform.rotation.x = q[1]; msgout.transform.rotation.y = q[2]; msgout.transform.rotation.z = q[3];
  msgout.transform.translation.x = 0.0; msgout.transform.translation.y = 0.0; msgout.transform.translation.z = 0.0;
  
  resetPublisher.publish(msgout);
}




/*
void CAMERACallback(const geometry_msgs::Pose::ConstPtr& camera_pose)
{
  ROS_INFO("I heard camera pose: [%f, %f, %f %f, %f, %f, %f]", camera_pose->position.x, camera_pose->position.y, camera_pose->position.z, 
  													camera_pose->orientation.x, camera_pose->orientation.y, camera_pose->orientation.z, camera_pose->orientation.w);
}

void IMUCallback(const geometry_msgs::Pose::ConstPtr& imu_pose)
{
  ROS_INFO("I heard imu pose: [%f, %f, %f %f, %f, %f, %f]", imu_pose->position.x, imu_pose->position.y, imu_pose->position.z, 
  													imu_pose->orientation.x, imu_pose->orientation.y, imu_pose->orientation.z, imu_pose->orientation.w);
}
*/

int main(int argc, char **argv)
{
	// system frequency
	const double frequency = 100.0;
	// initialize the kalman filter class
	KalmanFilter10 kalmanFilter; 
	KalmanOrientation KalmanOrientation;
	double q[4] = {0.0, 0.0, 0.0, 0.0}; //this has to be here, because it is shared between two callbacks   

	// ros initilize
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle n;

	// ros publisher init
	static tf::TransformBroadcaster imuPublisher;
	ros::Publisher           imuIndicator	      = n.advertise<visualization_msgs::Marker>("vi/imu/marker", 100);
	static tf::TransformBroadcaster visPublisher;
  	ros::Publisher           visIndicator	      = n.advertise<visualization_msgs::Marker>("vi/svo/marker", 100);
  	static tf::TransformBroadcaster kalmanPublisher;
	ros::Publisher           kalmanIndicator	  = n.advertise<visualization_msgs::Marker>("vi/kalman/marker", 100);
  	// ekf publisher 
  	ros::Publisher           ekfPublisher = n.advertise<kalman_filter::adafruit_msg>("ekf/output",1);

  	ros::Publisher           resetPublisher     = n.advertise<geometry_msgs::TransformStamped>("Allreset",10);
 	//ros::Publisher           mValidPublisher    = n.advertise<std_msgs::Int8>                 ("/ekf/safeout",100);
  	ros::Publisher           iValidPublisher    = n.advertise<geometry_msgs::Vector3>         ("ekf/safeoutVec",100);

  	// Initiate safe guard
  	ValidationGuard guard(&kalmanFilter, &resetPublisher, &iValidPublisher);
 
	// Subscriber init 
	// boost::ref - which passes its argument by reference 
  	ros::Subscriber imuSub = n.subscribe<geometry_msgs::TransformStamped>  ("adafruit/imu", 1, boost::bind(IMUCallback,
										              _1,
											      boost::ref(imuPublisher),
											      boost::ref(imuIndicator),
											      boost::ref(kalmanIndicator),
											      boost::ref(kalmanFilter),
											      boost::ref(KalmanOrientation),
											      boost::ref(q),
											      boost::ref(guard),
											      boost::ref(ekfPublisher)));	

  	ros::Subscriber exposureSub = n.subscribe<geometry_msgs::TransformStamped>  ("teensy/imu", 1, boost::bind(ExposureCallback,
										              _1,
											      boost::ref(imuPublisher),
											      boost::ref(imuIndicator),
											      boost::ref(kalmanFilter)));

  	ros::Subscriber visSub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("orbslam2/pose",  1, boost::bind(VISCallback,
												    _1,
												    boost::ref(visPublisher),
												    boost::ref(visIndicator),
												    boost::ref(kalmanIndicator),
												    boost::ref(kalmanFilter),
												    boost::ref(KalmanOrientation)));
	
	ros::Subscriber resSub = n.subscribe<geometry_msgs::TransformStamped>("Allreset",  10, boost::bind(resetCallback,
  													_1,
  													boost::ref(kalmanFilter),
													boost::ref(guard)));

	ros::Subscriber resSub_ros = n.subscribe< std_msgs::Bool>("svo/usereset",  10, boost::bind(SVOresetCallback,
  											_1,
  											boost::ref(kalmanFilter),
											boost::ref(resetPublisher),
											boost::ref(q)));
  	// for the validation guard
  	// guard.sub_svo_info_ = n.subscribe("svo/info",   1,  &ValidationGuard::svo_info_Cb, &guard);


/*	// ros subscriber  init 
	ros::Subscriber camera_sub = n.subscribe<geometry_msgs::Pose> ("camera/pose", 1, CAMERACallback);
	ros::Subscriber imu_sub = n.subscribe<geometry_msgs::Pose> ("imu/pose", 1, IMUCallback);

	ROS_INFO("I heard camera pose:");
*/

	ros::spin();
  	return 0;
}
