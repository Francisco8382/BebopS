/*
 * Copyright 2021 Francisco Javier Torres Maldonado
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Archivo basado en position_controller.cpp

#include "bebop_simulator/pos_controller_3.h"
#include "bebop_simulator/transform_datatypes.h"
#include "bebop_simulator/Matrix3x3.h"
#include "bebop_simulator/Quaternion.h" 
#include "bebop_simulator/stabilizer_types.h"

#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <ros/ros.h>
#include <chrono>
#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>

#include "bebop_simulator/ReferenceAngles.h"
#include "bebop_simulator/PosController.h"
#include "bebop_simulator/AttitudeController.h"


#define M_PI                      3.14159265358979323846  /* pi */
#define TsP                       10e-3  /* Position control sampling time */
#define TsA                       5e-3 /* Attitude control sampling time */
#define MAX_ROTOR_VELOCITY        1475 /* Max rotors velocity [rad/s] */
#define MIN_ROTOR_VELOCITY        0 /* Min rotors velocity [rad/s] */
#define POW_MAX_ROTOR_VELOCITY    MAX_ROTOR_VELOCITY*MAX_ROTOR_VELOCITY /* Squared max rotors velocity [rad/s] */

using namespace std;

double sgn(double _val){
  if (_val > 0){
    return 1;
  }
  else if (_val < 0){
    return -1;
  }
  else {
    return 0;
  }
}

double clamp(double _val, double _max){
  if (_val > 0){
    if (_val < std::abs(_max)){
      return _val;
    }
    else {
      return std::abs(_max);
    }
  }
  else {
    if (_val > -std::abs(_max)){
      return _val;
    }
    else {
      return -std::abs(_max);
    }
  }
}

namespace bebop_simulator {

PositionController::PositionController()
    : controller_active_(false),
      dataStoring_active_(false),
      waypointFilter_active_(true),
      EKF_active_(false),
      dataStoringTime_(0),
      wallSecsOffset_(0),
      e_x_(0),
      e_y_(0),
      e_z_(0),
      dot_e_x_(0),
      dot_e_y_(0), 
      dot_e_z_(0),
      dot_e_x_ant_(0),
      dot_e_y_ant_(0),
      dot_e_z_ant_(0),
      e_phi_(0),
      e_theta_(0),
      e_psi_(0),
      dot_e_phi_(0),
      dot_e_theta_(0), 
      dot_e_psi_(0),
      dot_e_phi_ant_(0),
      dot_e_theta_ant_(0),
      dot_e_psi_ant_(0),
      sum_zeta_x_(0),
      sum_zeta_y_(0),
      sum_zeta_z_(0),
      sum_eta_x_(0),
      sum_eta_y_(0),
      sum_eta_z_(0),
      sum_zeta_phi_(0),
      sum_zeta_theta_(0),
      sum_zeta_psi_(0),
      sum_eta_phi_(0),
      sum_eta_theta_(0),
      sum_eta_psi_(0),
      bf_(0),
      l_(0),
      bm_(0),
      m_(0),
      g_(0),
      Ix_(0),
      Iy_(0),
      Iz_(0),
      K1_xy_(0),
      K1_z_(0),
      K1_phi_theta_(0),
      K1_psi_(0),
      K2_xy_(0),
      K2_z_(0),
      K2_phi_theta_(0),
      K2_psi_(0),
      K3_xy_(0),
      K3_z_(0),
      K3_phi_theta_(0),
      K3_psi_(0),
      K4_xy_(0),
      K4_z_(0),
      K4_phi_theta_(0),
      K4_psi_(0),
      lambda_x_(0),
      lambda_y_(0),
      lambda_z_(0),
      sigma(0),
      control_({0,0,0,0}), //roll, pitch, yaw rate, thrust
      state_({0,  //Position.x 
              0,  //Position.y
              0,  //Position.z
              0,  //Linear velocity x
              0,  //Linear velocity y
              0,  //Linear velocity z
              0,  //Quaternion x
              0,  //Quaternion y
              0,  //Quaternion z
              0,  //Quaternion w
              0,  //Angular velocity x
              0,  //Angular velocity y
              0}) //Angular velocity z)
              {  

			          // Command signals initialization
            		command_trajectory_.setFromYaw(0);
            		command_trajectory_.position_W[0] = 0;
            		command_trajectory_.position_W[1] = 0;
            		command_trajectory_.position_W[2] = 0;

            		// Kalman filter's parameters initialization
            		filter_parameters_.dev_x_ = 0;
            		filter_parameters_.dev_y_ = 0;
            		filter_parameters_.dev_z_ = 0;
            		filter_parameters_.dev_vx_ = 0;
            		filter_parameters_.dev_vy_ = 0;
            		filter_parameters_.dev_vz_ = 0;
            		filter_parameters_.Qp_x_ = 0;
            		filter_parameters_.Qp_y_ = 0;
            		filter_parameters_.Qp_z_ = 0;
            		filter_parameters_.Qp_vx_ = 0;
            		filter_parameters_.Qp_vy_ = 0;
            		filter_parameters_.Qp_vz_ = 0;
            		filter_parameters_.Rp_ = Eigen::MatrixXf::Zero(6,6);
            		filter_parameters_.Qp_ = Eigen::MatrixXf::Identity(6,6);

            	  // Timers set the outer and inner loops working frequency
            		timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionController::CallbackAttitude, this, false, true);
            		timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionController::CallbackPosition, this, false, true);

                ref_angles_pub = nh.advertise<bebop_simulator::ReferenceAngles>("/ref_angles", 1);
                position_controller_pub = nh.advertise<bebop_simulator::PosController>("/pos_controller", 1);
                attitude_controller_pub = nh.advertise<bebop_simulator::AttitudeController>("/attitude_controller", 1);

                ROS_INFO("POS CONTROLLER");


}

//The library Destructor
PositionController::~PositionController() {}

//The callback saves data come from simulation into csv files
void PositionController::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream fileControllerGains;
      ofstream fileVehicleParameters;
      ofstream fileControlSignals;
      ofstream fileControlMixerTerms;
      ofstream filePropellersAngularVelocities;
      ofstream fileReferenceAngles;
      ofstream fileVelocityErrors;
      ofstream fileDroneAttiude;
      ofstream fileTrajectoryErrors;
      ofstream fileAttitudeErrors;
      ofstream fileDerivativeAttitudeErrors;
      ofstream fileTimeAttitudeErrors;
      ofstream fileTimePositionErrors;
      ofstream fileDroneAngularVelocitiesABC;
      ofstream fileDroneTrajectoryReference;
      ofstream fileControlMixerTermsSaturated;
      ofstream fileControlMixerTermsUnsaturated;
      ofstream fileDroneLinearVelocitiesABC;
      ofstream fileDronePosition;
      ofstream fileControlMixerUnSaturatedBefore;

      ROS_INFO("CallbackSavaData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec, odometry_.timeStampNsec);
    
      fileControllerGains.open("/home/" + user_ + "/controllerGains.csv", std::ios_base::app);
      fileVehicleParameters.open("/home/" + user_ + "/vehicleParameters.csv", std::ios_base::app);
      fileControlSignals.open("/home/" + user_ + "/controlSignals.csv", std::ios_base::app);
      fileControlMixerTerms.open("/home/" + user_ + "/controlMixer.csv", std::ios_base::app);
      filePropellersAngularVelocities.open("/home/" + user_ + "/propellersAngularVelocities.csv", std::ios_base::app);
      fileReferenceAngles.open("/home/" + user_ + "/referenceAngles.csv", std::ios_base::app);
      fileVelocityErrors.open("/home/" + user_ + "/velocityErrors.csv", std::ios_base::app);
      fileDroneAttiude.open("/home/" + user_ + "/droneAttitude.csv", std::ios_base::app);
      fileTrajectoryErrors.open("/home/" + user_ + "/trajectoryErrors.csv", std::ios_base::app);
      fileAttitudeErrors.open("/home/" + user_ + "/attitudeErrors.csv", std::ios_base::app);
      fileDerivativeAttitudeErrors.open("/home/" + user_ + "/derivativeAttitudeErrors.csv", std::ios_base::app);
      fileTimeAttitudeErrors.open("/home/" + user_ + "/timeAttitudeErrors.csv", std::ios_base::app);
      fileTimePositionErrors.open("/home/" + user_ + "/timePositionErrors.csv", std::ios_base::app);
      fileDroneAngularVelocitiesABC.open("/home/" + user_ + "/droneAngularVelocitiesABC.csv", std::ios_base::app);
      fileDroneTrajectoryReference.open("/home/" + user_ + "/droneTrajectoryReferences.csv", std::ios_base::app);
      fileControlMixerTermsSaturated.open("/home/" + user_ + "/controlMixerTermsSaturated.csv", std::ios_base::app);
      fileControlMixerTermsUnsaturated.open("/home/" + user_ + "/controlMixerTermsUnsaturated.csv", std::ios_base::app);
      fileDroneLinearVelocitiesABC.open("/home/" + user_ + "/droneLinearVelocitiesABC.csv", std::ios_base::app);
      fileDronePosition.open("/home/" + user_ + "/dronePosition.csv", std::ios_base::app);
      fileControlMixerUnSaturatedBefore.open("/home/" + user_ + "/controlMixerUnSaturatedBefore.csv", std::ios_base::app);

      // Saving vehicle parameters in a file
      fileControllerGains << K1_xy_ << "," << K1_z_ << "," << K1_phi_theta_ << "," << K1_psi_ << "," << K2_xy_ << ","
    		  << K2_z_ << "," << K2_phi_theta_ << "," << K2_psi_ << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      // Saving vehicle parameters in a file
      fileVehicleParameters << bf_ << "," << l_ << "," << bm_ << "," << m_ << "," << g_ << "," << Ix_ << "," << Iy_ << "," << Iz_ << ","
    		  << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      // Saving control signals in a file
      for (unsigned n=0; n < listControlSignals_.size(); ++n) {
          fileControlSignals << listControlSignals_.at( n );
      }

      // Saving the control mixer terms in a file
      for (unsigned n=0; n < listControlMixerTerms_.size(); ++n) {
          fileControlMixerTerms << listControlMixerTerms_.at( n );
      }

      // Saving the propellers angular velocities in a file
      for (unsigned n=0; n < listPropellersAngularVelocities_.size(); ++n) {
          filePropellersAngularVelocities << listPropellersAngularVelocities_.at( n );
      }

      // Saving the reference angles in a file
      for (unsigned n=0; n < listReferenceAngles_.size(); ++n) {
          fileReferenceAngles << listReferenceAngles_.at( n );
      }

      // Saving the velocity errors in a file
      for (unsigned n=0; n < listVelocityErrors_.size(); ++n) {
          fileVelocityErrors << listVelocityErrors_.at( n );
      }

      // Saving the drone attitude in a file
      for (unsigned n=0; n < listDroneAttitude_.size(); ++n) {
          fileDroneAttiude << listDroneAttitude_.at( n );
      }
 
      // Saving the trajectory errors in a file
      for (unsigned n=0; n < listTrajectoryErrors_.size(); ++n) {
          fileTrajectoryErrors << listTrajectoryErrors_.at( n );
      }

      // Saving the attitude errors in a file
      for (unsigned n=0; n < listAttitudeErrors_.size(); ++n) {
          fileAttitudeErrors << listAttitudeErrors_.at( n );
      }

      // Saving the derivative attitude errors in a file
      for (unsigned n=0; n < listDerivativeAttitudeErrors_.size(); ++n) {
          fileDerivativeAttitudeErrors << listDerivativeAttitudeErrors_.at( n );
      }

      // Saving the position and attitude errors along the time
      for (unsigned n=0; n < listTimeAttitudeErrors_.size(); ++n) {
          fileTimeAttitudeErrors << listTimeAttitudeErrors_.at( n );
      }

      // Saving
      for (unsigned n=0; n < listTimePositionErrors_.size(); ++n) {
          fileTimePositionErrors << listTimePositionErrors_.at( n );
      }

      // Saving the drone angular velocity in the aircraft body reference system
      for (unsigned n=0; n < listDroneAngularVelocitiesABC_.size(); ++n) {
    	  fileDroneAngularVelocitiesABC << listDroneAngularVelocitiesABC_.at( n );
      }

      // Saving the drone trajectory references (them coming from the waypoint filter)
      for (unsigned n=0; n < listDroneTrajectoryReference_.size(); ++n) {
    	  fileDroneTrajectoryReference << listDroneTrajectoryReference_.at( n );
      }

      // Saving the saturated and unsaturated values
      for (unsigned n=0; n < listControlMixerTermsSaturated_.size(); ++n) {
    	  fileControlMixerTermsSaturated << listControlMixerTermsSaturated_.at( n );
      }

      // Saving the saturated and unsaturated values
      for (unsigned n=0; n < listControlMixerTermsUnsaturated_.size(); ++n) {
    	  fileControlMixerTermsUnsaturated << listControlMixerTermsUnsaturated_.at( n );
      }

      // Saving drone linear velocity in the aircraft body center reference system
      for (unsigned n=0; n < listDroneLinearVelocitiesABC_.size(); ++n) {
    	  fileDroneLinearVelocitiesABC << listDroneLinearVelocitiesABC_.at( n );
      }

      // Saving the drone position along axes
      for (unsigned n=0; n < listDronePosition_.size(); ++n) {
        fileDronePosition << listDronePosition_.at( n );
      }

      // Saving not_saturated_1, not_saturated_2, not_saturated_3, not_saturated_4
      for (unsigned n=0; n < listControlMixerTermsUnSaturatedBefore_.size(); ++n) {
        fileControlMixerUnSaturatedBefore << listControlMixerTermsUnSaturatedBefore_.at( n );
      }

      // Closing all opened files
      fileControllerGains.close ();
      fileVehicleParameters.close ();
      fileControlSignals.close ();
      fileControlMixerTerms.close();
      filePropellersAngularVelocities.close();
      fileReferenceAngles.close();
      fileVelocityErrors.close();
      fileDroneAttiude.close();
      fileTrajectoryErrors.close();
      fileAttitudeErrors.close();
      fileDerivativeAttitudeErrors.close();
      fileTimeAttitudeErrors.close();
      fileTimePositionErrors.close();
      fileDroneAngularVelocitiesABC.close();
      fileDroneTrajectoryReference.close();
      fileControlMixerTermsSaturated.close();
      fileControlMixerTermsUnsaturated.close();
      fileDroneLinearVelocitiesABC.close();
      fileDronePosition.close();
      fileControlMixerUnSaturatedBefore.close();

      // To have a one shot storing
      dataStoring_active_ = false;

}

// The function moves the controller gains read from controller_bebop.yaml file to private variables of the class.
// These variables will be employed during the simulation
void PositionController::SetControllerGains(){

      K1_xy_ = controller_parameters_.K1_xy_;
      K1_z_ = controller_parameters_.K1_z_;

      K1_phi_theta_ = controller_parameters_.K1_phi_theta_;
      K1_psi_ = controller_parameters_.K1_psi_;

      K2_xy_ = controller_parameters_.K2_xy_;
      K2_z_ = controller_parameters_.K2_z_;

      K2_phi_theta_ = controller_parameters_.K2_phi_theta_;
      K2_psi_ = controller_parameters_.K2_psi_;

      K3_xy_ = controller_parameters_.K3_xy_;
      K3_z_ = controller_parameters_.K3_z_;

      K3_phi_theta_ = controller_parameters_.K3_phi_theta_;
      K3_psi_ = controller_parameters_.K3_psi_;

      K4_xy_ = controller_parameters_.K4_xy_;
      K4_z_ = controller_parameters_.K4_z_;

      K4_phi_theta_ = controller_parameters_.K4_phi_theta_;
      K4_psi_ = controller_parameters_.K4_psi_;

      lambda_x_ = controller_parameters_.U_q_.x();
      lambda_y_ = controller_parameters_.U_q_.y();
      lambda_z_ = controller_parameters_.U_q_.z();

      sigma = controller_parameters_.sigma_;

}

// As SetControllerGains, the function is used to set the vehicle parameters into private variables of the class
void PositionController::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

      // On the EKF object is invoked the method SeVehicleParameters. Such function allows to send the vehicle parameters to the EKF class.
      // Then, they are employed to set the filter matrices
      extended_kalman_filter_bebop_.SetVehicleParameters(m_, g_);

}

// Reading parameters come frame launch file
void PositionController::SetLaunchFileParameters(){

	// The boolean variable is used to inactive the logging if it is not useful
	if(dataStoring_active_){

		// Time after which the data storing function is turned on
		timer3_ = n3_.createTimer(ros::Duration(dataStoringTime_), &PositionController::CallbackSaveData, this, false, true);

		// Cleaning the string vector contents
		listControlSignals_.clear();
		listControlSignals_.clear();
		listControlMixerTerms_.clear();
		listPropellersAngularVelocities_.clear();
		listReferenceAngles_.clear();
		listVelocityErrors_.clear();
		listDroneAttitude_.clear();
		listTrajectoryErrors_.clear();
		listAttitudeErrors_.clear();
		listDerivativeAttitudeErrors_.clear();
		listTimeAttitudeErrors_.clear();
		listTimePositionErrors_.clear();
	  listDroneAngularVelocitiesABC_.clear();
	  listDroneTrajectoryReference_.clear();
	  listControlMixerTermsSaturated_.clear();
	  listControlMixerTermsUnsaturated_.clear();
	  listDronePosition_.clear();
	  listControlMixerTermsUnSaturatedBefore_.clear();

		// The client needed to get information about the Gazebo simulation environment both the attitude and position errors
		clientAttitude_ = clientHandleAttitude_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
		clientPosition_ = clientHandlePosition_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

		ros::WallTime beginWallOffset = ros::WallTime::now();
		wallSecsOffset_ = beginWallOffset.toSec();

	}

}

void PositionController::SetFilterParameters(){

    // The function is used to move the filter parameters from the YAML file to the filter class
    extended_kalman_filter_bebop_.SetFilterParameters(&filter_parameters_);

}

// The functions is used to convert the drone attitude from quaternion to Euler angles
void PositionController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    double x, y, z, w;
    x = odometry_.orientation.x();
    y = odometry_.orientation.y();
    z = odometry_.orientation.z();
    w = odometry_.orientation.w();
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);
	
}

// When a new odometry message comes, the content of the message is stored in private variable. At the same time, the controller is going to be active.
// The attitude of the aircraft is computer (as we said before it move from quaterninon to Euler angles) and also the angular velocity is stored.
void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 
    controller_active_= true;

    Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

}

// The function allows to set the waypoint filter parameters
void PositionController::SetWaypointFilterParameters(){

  waypoint_filter_.SetParameters(&waypoint_filter_parameters_);

}

// The function sets the filter trajectory points
void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode) {

  // If the waypoint has been activated or not
  if(waypointFilter_active_){
    waypoint_filter_.SetTrajectoryPoint(command_trajectory_positionControllerNode);
  }
  else
    command_trajectory_ = command_trajectory_positionControllerNode;


}

// Just to plot the data during the simulation
void PositionController::GetTrajectory(nav_msgs::Odometry* smoothed_trajectory){

   smoothed_trajectory->pose.pose.position.x = command_trajectory_.position_W[0];
   smoothed_trajectory->pose.pose.position.y = command_trajectory_.position_W[1];
   smoothed_trajectory->pose.pose.position.z = command_trajectory_.position_W[2];

}

// Just to plot the data during the simulation
void PositionController::GetOdometry(nav_msgs::Odometry* odometry_filtered){

   *odometry_filtered = odometry_filtered_private_;

}

// Just to analyze the position and velocity errors
void PositionController::GetPositionAndVelocityErrors(nav_msgs::Odometry* positionAndVelocityErrors){

  positionAndVelocityErrors->pose.pose.position.x = e_x_;
  positionAndVelocityErrors->pose.pose.position.y = e_y_;
  positionAndVelocityErrors->pose.pose.position.z = e_z_;

  positionAndVelocityErrors->twist.twist.linear.x = dot_e_x_;
  positionAndVelocityErrors->twist.twist.linear.y = dot_e_y_;
  positionAndVelocityErrors->twist.twist.linear.z = dot_e_z_;

}

// Just to analyze the attitude and angular velocity errors
void PositionController::GetAngularAndAngularVelocityErrors(nav_msgs::Odometry* angularAndAngularVelocityErrors){

  angularAndAngularVelocityErrors->pose.pose.position.x = e_phi_;
  angularAndAngularVelocityErrors->pose.pose.position.y = e_theta_;
  angularAndAngularVelocityErrors->pose.pose.position.z = e_psi_;

  angularAndAngularVelocityErrors->twist.twist.linear.x = dot_e_phi_;
  angularAndAngularVelocityErrors->twist.twist.linear.y = dot_e_theta_;
  angularAndAngularVelocityErrors->twist.twist.linear.z = dot_e_psi_;

}

// Just to plot the data during the simulation
void PositionController::GetReferenceAngles(nav_msgs::Odometry* reference_angles){
   assert(reference_angles);

   double u_x, u_y, u_z, u_Terr;
   PosController(&control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);

   reference_angles->pose.pose.position.x = control_.phiR*180/M_PI;
   reference_angles->pose.pose.position.y = control_.thetaR*180/M_PI;
   reference_angles->pose.pose.position.z = control_.uT;

   reference_angles->twist.twist.linear.x = u_x;
   reference_angles->twist.twist.linear.y = u_y;
   reference_angles->twist.twist.linear.z = u_Terr;

}

// Just to plot the components make able to compute dot_e_z
void PositionController::GetVelocityAlongZComponents(nav_msgs::Odometry* zVelocity_components){
  assert(zVelocity_components);

  zVelocity_components->pose.pose.position.x = state_.linearVelocity.x;
  zVelocity_components->pose.pose.position.y = state_.linearVelocity.y;
  zVelocity_components->pose.pose.position.z = state_.linearVelocity.z;

  double phi, theta, psi;
  Quaternion2Euler(&phi, &theta, &psi);

  zVelocity_components->twist.twist.linear.x = (-sin(theta) * state_.linearVelocity.x);
  zVelocity_components->twist.twist.linear.y = ( sin(phi) * cos(theta) * state_.linearVelocity.y);
  zVelocity_components->twist.twist.linear.z = (cos(phi) * cos(theta) * state_.linearVelocity.z);

}

// The functions is used to get information about the estimated state when bias and noise affect both the accelerometer and angular rate measurements
void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.SetThrustCommand(control_.uT);

    //The EKF works or not in according to the value of the EKF_active_ variables
    if(EKF_active_)
    	extended_kalman_filter_bebop_.EstimatorWithNoise(&state_, &odometry_, &odometry_filtered_private_);
    else
    	extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);

}

// The function computes the propellers angular velocity
void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    // The controller is inactive if a point to reach is not coming
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double u_phi, u_theta, u_psi;
    double u_x, u_y, u_z, u_Terr;
    AttitudeController(&u_phi, &u_theta, &u_psi);
	  PosController(&control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);

	  // Data storing section. It is actived if necessary
    if(dataStoring_active_){
      
      // Saving drone attitude in a file
      std::stringstream tempDroneAttitude;
      tempDroneAttitude << state_.attitude.roll << "," << state_.attitude.pitch << "," << state_.attitude.yaw << ","
          <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listDroneAttitude_.push_back(tempDroneAttitude.str());

      // Saving the drone angular velocity in the aircraft body reference system
      std:stringstream tempDroneAngularVelocitiesABC;
      tempDroneAngularVelocitiesABC << state_.angularVelocity.x << "," << state_.angularVelocity.y << "," << state_.angularVelocity.z
        << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listDroneAngularVelocitiesABC_.push_back(tempDroneAngularVelocitiesABC.str());
    }
    
    double first, second, third, fourth;
    first = (1 / ( 4 * bf_ )) * control_.uT;
    second = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_phi;
    third = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_theta;
    fourth = (1 / ( 4 * bf_ * bm_)) * u_psi;

	
    if(dataStoring_active_){
      //Saving the control mixer terms in a file
      std::stringstream tempControlMixerTerms;
      tempControlMixerTerms << first << "," << second << "," << third << "," << fourth << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listControlMixerTerms_.push_back(tempControlMixerTerms.str());
    }

    double not_saturated_1, not_saturated_2, not_saturated_3, not_saturated_4;
    not_saturated_1 = first - second - third - fourth;
    not_saturated_2 = first + second - third + fourth;
    not_saturated_3 = first + second + third - fourth;
    not_saturated_4 = first - second + third + fourth;

    if(dataStoring_active_){
      // Saving the unsaturated values before the limiting
      std::stringstream tempControlMixerTermsUnSaturatedBefore;
      tempControlMixerTermsUnSaturatedBefore << not_saturated_1 << "," << not_saturated_2 << "," << not_saturated_3 << "," << not_saturated_4 << ","
          << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listControlMixerTermsUnSaturatedBefore_.push_back(tempControlMixerTermsUnSaturatedBefore.str());
    }

    // The propellers velocities is limited by taking into account the physical constrains
    double motorMin=not_saturated_1, motorMax=not_saturated_1, motorFix=0;

    if(not_saturated_2 < motorMin) motorMin = not_saturated_2;
    if(not_saturated_2 > motorMax) motorMax = not_saturated_2;

    if(not_saturated_3 < motorMin) motorMin = not_saturated_3;
    if(not_saturated_3 > motorMax) motorMax = not_saturated_3;

    if(not_saturated_4 < motorMin) motorMin = not_saturated_4;
    if(not_saturated_4 > motorMax) motorMax = not_saturated_4;

    if(motorMin < MIN_ROTOR_VELOCITY) motorFix = MIN_ROTOR_VELOCITY - motorMin;
    else if(motorMax > POW_MAX_ROTOR_VELOCITY) motorFix = POW_MAX_ROTOR_VELOCITY - motorMax;

    not_saturated_1 = not_saturated_1 + motorFix;
    not_saturated_2 = not_saturated_2 + motorFix;
    not_saturated_3 = not_saturated_3 + motorFix;
    not_saturated_4 = not_saturated_4 + motorFix;

    // The values have been saturated to avoid the root square of negative values
    double saturated_1, saturated_2, saturated_3, saturated_4;
    if(not_saturated_1 < 0)
      saturated_1 = 0;
    else if(not_saturated_1 > POW_MAX_ROTOR_VELOCITY)
      saturated_1 = POW_MAX_ROTOR_VELOCITY;
    else
      saturated_1 = not_saturated_1;

    if(not_saturated_2 < 0)
      saturated_2 = 0;
    else if(not_saturated_2 > POW_MAX_ROTOR_VELOCITY)
      saturated_2 = POW_MAX_ROTOR_VELOCITY;
    else
      saturated_2 = not_saturated_2;

    if(not_saturated_3 < 0)
      saturated_3 = 0;
    else if(not_saturated_3 > POW_MAX_ROTOR_VELOCITY)
      saturated_3 = POW_MAX_ROTOR_VELOCITY;
    else
      saturated_3 = not_saturated_3;

    if(not_saturated_4 < 0)
      saturated_4 = 0;
    else if(not_saturated_4 > POW_MAX_ROTOR_VELOCITY)
      saturated_4 = POW_MAX_ROTOR_VELOCITY;
    else
      saturated_4 = not_saturated_4;

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = sqrt(saturated_1);
    omega_2 = sqrt(saturated_2);
    omega_3 = sqrt(saturated_3);
    omega_4 = sqrt(saturated_4);


    if(dataStoring_active_){
      //Saving the saturated and unsaturated values
      std::stringstream tempControlMixerTermsSaturated;
      tempControlMixerTermsSaturated << saturated_1 << "," << saturated_2 << "," << saturated_3 << "," << saturated_4 << "," << odometry_.timeStampSec << ","
          << odometry_.timeStampNsec << "\n";

      listControlMixerTermsSaturated_.push_back(tempControlMixerTermsSaturated.str());

      std::stringstream tempControlMixerTermsUnsaturated;
      tempControlMixerTermsUnsaturated << not_saturated_1 << "," << not_saturated_2 << "," << not_saturated_3 << "," << not_saturated_4 << ","
          << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listControlMixerTermsUnsaturated_.push_back(tempControlMixerTermsUnsaturated.str());

      //Saving propellers angular velocities in a file
      std::stringstream tempPropellersAngularVelocities;
      tempPropellersAngularVelocities << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPropellersAngularVelocities_.push_back(tempPropellersAngularVelocities.str());
    }
    
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);

}

// The function computes the velocity errors
void PositionController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   // WITH THE EXTENDED KALMAN FILTER
   if(EKF_active_){
	   *dot_e_x = state_.linearVelocity.x - command_trajectory_.velocity_W[0];
	   *dot_e_y = state_.linearVelocity.y - command_trajectory_.velocity_W[1];
	   *dot_e_z = state_.linearVelocity.z - command_trajectory_.velocity_W[2];
   }
   else{
	   // The linear velocities are expressed in the inertial body frame.
	   double dot_x, dot_y, dot_z, theta, psi, phi;

	   theta = state_.attitude.pitch;
	   psi = state_.attitude.yaw;
	   phi = state_.attitude.roll;

	   dot_x = (cos(theta) * cos(psi) * state_.linearVelocity.x) +
	           ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * state_.linearVelocity.y) +
	           ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) *  state_.linearVelocity.z);

	   dot_y = (cos(theta) * sin(psi) * state_.linearVelocity.x) +
	           ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * state_.linearVelocity.y) +
	           ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) *  state_.linearVelocity.z);

	   dot_z = (-sin(theta) * state_.linearVelocity.x) + ( sin(phi) * cos(theta) * state_.linearVelocity.y) +
	           (cos(phi) * cos(theta) * state_.linearVelocity.z);

	   *dot_e_x = dot_x - command_trajectory_.velocity_W[0];
   	 *dot_e_y = dot_y - command_trajectory_.velocity_W[1];
   	 *dot_e_z = dot_z - command_trajectory_.velocity_W[2];

   }

   if(dataStoring_active_){
     // Saving drone linear velocity in the aircraft body center reference system
     std::stringstream tempDroneLinearVelocitiesABC;
     tempDroneLinearVelocitiesABC << state_.linearVelocity.x << "," << state_.linearVelocity.y << "," << state_.linearVelocity.z << ","
         << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

     listDroneLinearVelocitiesABC_.push_back(tempDroneLinearVelocitiesABC.str());
   }

}

// The function computes the position errors
void PositionController::PositionErrors(double* e_x, double* e_y, double* e_z){
   assert(e_x);
   assert(e_y); 
   assert(e_z);
   
   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];

   *e_x = state_.position.x - x_r;
   *e_y = state_.position.y - y_r;
   *e_z = state_.position.z - z_r;

}

// The function computes the position controller outputs
void PositionController::PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr){
   assert(u_T);
   assert(phi_r);
   assert(theta_r);
   assert(u_x);
   assert(u_y);
   assert(u_z);
   assert(u_Terr);
   
   //double sigma = 0.9;

   double Kp_xy_ = 17.8;
   double Kp_z_ = 75.5;
   double Td_xy_ = 1.88;
   double Td_z_ = 4.5;


   /*
   //u_x computing
   *u_x = -Kp_xy_*(e_x_ + dot_e_x_/Td_xy_)/lambda_x_;
   
   if (*u_x > 1 || *u_x <-1)
	   if (*u_x > 1)
		   *u_x = 1;
	   else
		   *u_x = -1;
	   
   *u_x = m_ * (*u_x * lambda_x_);
   
   //u_y computing
   *u_y = -Kp_xy_*(e_y_ + dot_e_y_/Td_xy_)/lambda_y_;
   
   if (*u_y > 1 || *u_y <-1)
	   if (*u_y > 1)
		   *u_y = 1;
	   else
		   *u_y = -1;
	   
   *u_y = m_* ( *u_y * lambda_y_);

   
   //u_z computing
   *u_z = -Kp_z_*(e_z_ + dot_e_z_/Td_z_)/lambda_z_;

   if (*u_z > 1 || *u_z <-1)
	   if (*u_z > 1)
		   *u_z = 1;
	   else
		   *u_z = -1;
	   
   *u_z = m_* ( *u_z * lambda_z_);

   */ 

   double v_x_nom = -K1_xy_*pow(std::abs(dot_e_x_),sigma)*sgn(dot_e_x_) - K2_xy_*pow(std::abs(e_x_),((sigma)/(2-sigma)))*sgn(e_x_);
   double v_y_nom = -K1_xy_*pow(std::abs(dot_e_y_),sigma)*sgn(dot_e_y_) - K2_xy_*pow(std::abs(e_y_),((sigma)/(2-sigma)))*sgn(e_y_);
   double v_z_nom = -K1_z_ *pow(std::abs(dot_e_z_),sigma)*sgn(dot_e_z_) - K2_z_ *pow(std::abs(e_z_),((sigma)/(2-sigma)))*sgn(e_z_);

   double v_x_stc = -K3_xy_*std::abs(sum_zeta_x_)*sgn(sum_zeta_x_) + sum_eta_x_;
   double v_y_stc = -K3_xy_*std::abs(sum_zeta_y_)*sgn(sum_zeta_y_) + sum_eta_y_;
   double v_z_stc = -K3_z_ *std::abs(sum_zeta_z_)*sgn(sum_zeta_z_) + sum_eta_z_;

   *u_x = clamp(v_x_nom+v_x_stc+command_trajectory_.acceleration_W[0],m_*(lambda_x_-std::abs(command_trajectory_.acceleration_W[0])));
   *u_y = clamp(v_y_nom+v_y_stc+command_trajectory_.acceleration_W[1],m_*(lambda_y_-std::abs(command_trajectory_.acceleration_W[1])));
   *u_z = clamp(v_z_nom+v_z_stc+command_trajectory_.acceleration_W[2],m_*(lambda_z_-std::abs(command_trajectory_.acceleration_W[2])));

   //*u_x = clamp(v_x_nom+v_x_stc-command_trajectory_.acceleration_W[0],m_*lambda_x_);
   //*u_y = clamp(v_y_nom+v_y_stc-command_trajectory_.acceleration_W[1],m_*lambda_y_);
   //*u_z = clamp(v_z_nom+v_z_stc-command_trajectory_.acceleration_W[2],m_*lambda_z_);

   //*u_x = clamp(v_x_nom+v_x_stc,m_*(lambda_x_-std::abs(command_trajectory_.acceleration_W[0])));
   //*u_y = clamp(v_y_nom+v_y_stc,m_*(lambda_y_-std::abs(command_trajectory_.acceleration_W[1])));
   //*u_z = clamp(v_z_nom+v_z_stc,m_*(lambda_z_-std::abs(command_trajectory_.acceleration_W[2])));
   
   //u_Terr computing
   *u_Terr = *u_z + (m_ * g_);
   
   //u_T computing
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );

   position_controller.u_x = *u_x;
   position_controller.u_y = *u_y;
   position_controller.u_z = *u_z;
   position_controller.u_T = *u_T;
   position_controller_pub.publish(position_controller);

   double psi_r;
   psi_r = command_trajectory_.getYaw();

   *theta_r = atan( ( (*u_x * cos(psi_r) ) + ( *u_y * sin(psi_r) ) )  / *u_Terr );

   *phi_r = atan( cos(*theta_r) * ( ( (*u_x * sin(psi_r)) - (*u_y * cos(psi_r)) ) / (*u_Terr) ) );

   ref_angles.theta = *theta_r;
   ref_angles.phi = *phi_r;
   ref_angles_pub.publish(ref_angles);

   //ros::param::set("/theta_ref", theta_ref);
   //ros::param::set("/phi_ref", phi_ref);

   if(dataStoring_active_){
      //Saving reference angles in a file
      std::stringstream tempReferenceAngles;
      tempReferenceAngles << *theta_r << "," << *phi_r << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listReferenceAngles_.push_back(tempReferenceAngles.str());
	  
	    double u_phi, u_theta, u_psi;
	    AttitudeController(&u_phi, &u_theta, &u_psi);
	  
	    //Saving control signals in a file
      std::stringstream tempControlSignals;
      tempControlSignals << *u_T << "," << u_phi << "," << u_theta << "," << u_psi << "," << *u_x << "," << *u_y << ","
          << *u_Terr << "," << *u_z << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listControlSignals_.push_back(tempControlSignals.str());

    }

}

//The function computes the attitude errors
void PositionController::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
   assert(e_phi);
   assert(e_theta);
   assert(e_psi);
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double u_T, u_x, u_y, u_z, u_Terr;
   PosController(&u_T, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);

   *e_phi = state_.attitude.roll - control_.phiR;
   *e_theta = state_.attitude.pitch - control_.thetaR;
   *e_psi = state_.attitude.yaw - psi_r;

   // Modificaciones
   /*
   while (abs(*e_psi) > M_PI) {
     if (*e_psi > 0) {
       *e_psi = *e_psi - 2*M_PI;
     } else {
       *e_psi = *e_psi + 2*M_PI;
     }
   }
   */

}

//The angular velocity errors
void PositionController::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
   assert(dot_e_phi);
   assert(dot_e_theta);
   assert(dot_e_psi);

   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double dot_phi, dot_theta, dot_psi;

   dot_phi = state_.angularVelocity.x + (sin(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.y)
                + (cos(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.z);
    
   dot_theta = (cos(state_.attitude.roll) * state_.angularVelocity.y) - (sin(state_.attitude.roll) * state_.angularVelocity.z);

   dot_psi = ( ( sin(state_.attitude.roll) * state_.angularVelocity.y) / cos(state_.attitude.pitch) ) +
		         ( ( cos(state_.attitude.roll) * state_.angularVelocity.z) / cos(state_.attitude.pitch) );

   *dot_e_phi =  dot_phi;
   *dot_e_theta = dot_theta;
   *dot_e_psi = dot_psi;

}

//The function computes the attitude controller outputs
void PositionController::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   double mu_phi_ = 0.0544;
   double mu_theta_ = 0.0543;
   double mu_psi_ = 0.44;

   double Kp_phi_theta_ = 578.85;
   double Kp_psi_ = 23.15;
   double Td_phi_theta_ = 9.15;
   double Td_psi_ = 3.93;

   //double sigma = 0.95;

   //*u_phi = Ix_ *   ( Kp_phi_theta_ * ( ( -dot_e_phi_   / Td_phi_theta_ ) - e_phi_   ) - ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * -e_theta_ * -e_psi_) );
   //*u_theta = Iy_ * ( Kp_phi_theta_ * ( ( -dot_e_theta_ / Td_phi_theta_ ) - e_theta_ ) - ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_  ) ) * -e_phi_   * -e_psi_) );
   //*u_psi = Iz_ *   ( Kp_psi_       * ( ( -dot_e_psi_   / Td_psi_       ) - e_psi_   ) - ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * -e_theta_ * -e_phi_) );

   //*u_phi =   -K1_phi_theta_*pow(std::abs(dot_e_phi_  ),sigma)*sgn(dot_e_phi_  ) - K2_phi_theta_*pow(std::abs(e_phi_  ),((sigma)/(2-sigma)))*sgn(e_phi_  );
   //*u_theta = -K1_phi_theta_*pow(std::abs(dot_e_theta_),sigma)*sgn(dot_e_theta_) - K2_phi_theta_*pow(std::abs(e_theta_),((sigma)/(2-sigma)))*sgn(e_theta_);
   //*u_psi =   -K1_psi_      *pow(std::abs(dot_e_psi_  ),sigma)*sgn(dot_e_psi_  ) - K2_psi_      *pow(std::abs(e_psi_  ),((sigma)/(2-sigma)))*sgn(e_psi_  );

   double v_phi_nom   = -K1_phi_theta_*pow(std::abs(dot_e_phi_  ),sigma)*sgn(dot_e_phi_  ) - K2_phi_theta_*pow(std::abs(e_phi_  ),((sigma)/(2-sigma)))*sgn(e_phi_  );
   double v_theta_nom = -K1_phi_theta_*pow(std::abs(dot_e_theta_),sigma)*sgn(dot_e_theta_) - K2_phi_theta_*pow(std::abs(e_theta_),((sigma)/(2-sigma)))*sgn(e_theta_);
   double v_psi_nom   = -K1_psi_      *pow(std::abs(dot_e_psi_  ),sigma)*sgn(dot_e_psi_  ) - K2_psi_      *pow(std::abs(e_psi_  ),((sigma)/(2-sigma)))*sgn(e_psi_  );

   double v_phi_stc   = -K3_phi_theta_*std::abs(sum_zeta_phi_  )*sgn(sum_zeta_phi_  ) + sum_eta_phi_;
   double v_theta_stc = -K3_phi_theta_*std::abs(sum_zeta_theta_)*sgn(sum_zeta_theta_) + sum_eta_theta_;
   double v_psi_stc   = -K3_psi_      *std::abs(sum_zeta_psi_  )*sgn(sum_zeta_psi_  ) + sum_eta_psi_;

   *u_phi   = v_phi_nom   + v_phi_stc   - ( ( (Iy_ - Iz_)/(mu_theta_ * mu_psi_) ) * -e_theta_ * -e_psi_);
   *u_theta = v_theta_nom + v_theta_stc - ( ( (Iz_ - Ix_)/(mu_phi_   * mu_psi_) ) * -e_phi_   * -e_psi_);
   *u_psi   = v_psi_nom   + v_psi_stc   - ( ( (Ix_ - Iy_)/(mu_theta_ * mu_phi_) ) * -e_theta_ * -e_phi_);

   attitude_controller.u_phi = *u_phi;
   attitude_controller.u_theta = *u_theta;
   attitude_controller.u_psi = *u_psi;
   attitude_controller_pub.publish(attitude_controller);

}


//The function every TsA computes the attitude and angular velocity errors. When the data storing is active, the data are saved into csv files
void PositionController::CallbackAttitude(const ros::TimerEvent& event){
     
     AttitudeErrors(&e_phi_, &e_theta_, &e_psi_);
     AngularVelocityErrors(&dot_e_phi_, &dot_e_theta_, &dot_e_psi_);

     double ddot_e_phi_   = (dot_e_phi_   - dot_e_phi_ant_  )/TsA;
     double ddot_e_theta_ = (dot_e_theta_ - dot_e_theta_ant_)/TsA;
     double ddot_e_psi_   = (dot_e_psi_   - dot_e_psi_ant_  )/TsA;

     dot_e_phi_ant_   = dot_e_phi_;
     dot_e_theta_ant_ = dot_e_theta_;
     dot_e_psi_ant_   = dot_e_psi_;

     double v_phi   = -K1_phi_theta_*pow(std::abs(dot_e_phi_  ),sigma)*sgn(dot_e_phi_  ) - K2_phi_theta_*pow(std::abs(e_phi_  ),((sigma)/(2-sigma)))*sgn(e_phi_  );
     double v_theta = -K1_phi_theta_*pow(std::abs(dot_e_theta_),sigma)*sgn(dot_e_theta_) - K2_phi_theta_*pow(std::abs(e_theta_),((sigma)/(2-sigma)))*sgn(e_theta_);
     double v_psi   = -K1_psi_      *pow(std::abs(dot_e_psi_  ),sigma)*sgn(dot_e_psi_  ) - K2_psi_      *pow(std::abs(e_psi_  ),((sigma)/(2-sigma)))*sgn(e_psi_  );

     double zeta_phi   = ddot_e_phi_   - v_phi;
     double zeta_theta = ddot_e_theta_ - v_theta;
     double zeta_psi   = ddot_e_psi_   - v_psi;

     sum_zeta_phi_   += zeta_phi   * TsA;
     sum_zeta_theta_ += zeta_theta * TsA;
     sum_zeta_psi_   += zeta_psi   * TsA;

     double eta_phi   = -K4_phi_theta_ * sgn(sum_zeta_phi_  );
     double eta_theta = -K4_phi_theta_ * sgn(sum_zeta_theta_);
     double eta_psi   = -K4_psi_       * sgn(sum_zeta_psi_  );

     sum_eta_phi_   += eta_phi   * TsA;
     sum_eta_theta_ += eta_theta * TsA;
     sum_eta_psi_   += eta_psi   * TsA;

     //Saving the time instant when the attitude errors are computed
     if(dataStoring_active_){	
        clientAttitude_.call(my_messageAttitude_);

        std::stringstream tempTimeAttitudeErrors;
        tempTimeAttitudeErrors << my_messageAttitude_.response.sim_time << "\n";
        listTimeAttitudeErrors_.push_back(tempTimeAttitudeErrors.str());

        ros::WallTime beginWall = ros::WallTime::now();
        double wallSecs = beginWall.toSec() - wallSecsOffset_;

        ros::Time begin = ros::Time::now();
        double secs = begin.toSec();

        //Saving attitude derivate errors in a file
        std::stringstream tempDerivativeAttitudeErrors;
        tempDerivativeAttitudeErrors << dot_e_phi_ << "," << dot_e_theta_ << "," << dot_e_psi_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << ","
            << my_messageAttitude_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listDerivativeAttitudeErrors_.push_back(tempDerivativeAttitudeErrors.str());

        //Saving attitude errors in a file
        std::stringstream tempAttitudeErrors;
        tempAttitudeErrors << e_phi_ << "," << e_theta_ << "," << e_psi_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << ","
            << my_messageAttitude_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listAttitudeErrors_.push_back(tempAttitudeErrors.str());

        //Saving the drone position along axes
        std::stringstream tempDronePosition;
        tempDronePosition << state_.position.x << "," << state_.position.y << "," << state_.position.z << ","
            << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listDronePosition_.push_back(tempDronePosition.str());

      }
}

//The function every TsP:
//	* the next point to follow has generated (the output of the waypoint filter)
//	* the output of the waypoint filter is put into the command_trajectory_ data structure
//  * the EKF is used to estimate the drone attitude and linear velocity
//  * the position and velocity errors are computed
//  * the last part is used to store the data into csv files if the data storing is active
void PositionController::CallbackPosition(const ros::TimerEvent& event){
  
     // The function is used to invoke the waypoint filter employs to reduce the error dimension along the axes when the drone stars to follow the trajectory.
     // The waypoint filter works with an update time of Tsp
     if(controller_active_)
       waypoint_filter_.Initialize(state_);

     if(waypointFilter_active_ && controller_active_){
        waypoint_filter_.TrajectoryGeneration();
        waypoint_filter_.GetTrajectoryPoint(&command_trajectory_);
     }

     SetOdometryEstimated();
     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);
     
     double ddot_e_x_ = (dot_e_x_ - dot_e_x_ant_)/TsP;
     double ddot_e_y_ = (dot_e_y_ - dot_e_y_ant_)/TsP;
     double ddot_e_z_ = (dot_e_z_ - dot_e_z_ant_)/TsP;

     dot_e_x_ant_ = dot_e_x_;
     dot_e_y_ant_ = dot_e_y_;
     dot_e_z_ant_ = dot_e_z_;

     double v_x = clamp(-K1_xy_*pow(std::abs(dot_e_x_),sigma)*sgn(dot_e_x_) - K2_xy_*pow(std::abs(e_x_),((sigma)/(2-sigma)))*sgn(e_x_),m_*lambda_x_);
     double v_y = clamp(-K1_xy_*pow(std::abs(dot_e_y_),sigma)*sgn(dot_e_y_) - K2_xy_*pow(std::abs(e_y_),((sigma)/(2-sigma)))*sgn(e_y_),m_*lambda_y_);
     double v_z = clamp(-K1_z_ *pow(std::abs(dot_e_z_),sigma)*sgn(dot_e_z_) - K2_z_ *pow(std::abs(e_z_),((sigma)/(2-sigma)))*sgn(e_z_),m_*lambda_z_);

     double zeta_x = ddot_e_x_ - v_x;
     double zeta_y = ddot_e_y_ - v_y;
     double zeta_z = ddot_e_z_ - v_z;

     sum_zeta_x_ += zeta_x * TsP;
     sum_zeta_y_ += zeta_y * TsP;
     sum_zeta_z_ += zeta_z * TsP;

     double eta_x = -K4_xy_*sgn(sum_zeta_x_);
     double eta_y = -K4_xy_*sgn(sum_zeta_y_);
     double eta_z = -K4_z_ *sgn(sum_zeta_z_);

     sum_eta_x_ += eta_x * TsP;
     sum_eta_y_ += eta_y * TsP;
     sum_eta_z_ += eta_z * TsP;

     //Saving the time instant when the position errors are computed
     if(dataStoring_active_){
        clientPosition_.call(my_messagePosition_);

        std::stringstream tempTimePositionErrors;
        tempTimePositionErrors << my_messagePosition_.response.sim_time << "\n";
        listTimePositionErrors_.push_back(tempTimePositionErrors.str());

        ros::WallTime beginWall = ros::WallTime::now();
        double wallSecs = beginWall.toSec() - wallSecsOffset_;

        ros::Time begin = ros::Time::now();
        double secs = begin.toSec();

        //Saving velocity errors in a file
        std::stringstream tempVelocityErrors;
        tempVelocityErrors << dot_e_x_ << "," << dot_e_y_ << "," << dot_e_z_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << ","
            << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listVelocityErrors_.push_back(tempVelocityErrors.str());

        //Saving trajectory errors in a file
        std::stringstream tempTrajectoryErrors;
        tempTrajectoryErrors << e_x_ << "," << e_y_ << "," << e_z_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << ","
            << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listTrajectoryErrors_.push_back(tempTrajectoryErrors.str());

        //Saving the drone trajectory references (them coming from the waypoint filter)
        std::stringstream tempTrajectoryReferences;
        tempTrajectoryReferences << command_trajectory_.position_W[0] << "," << command_trajectory_.position_W[1] << "," << command_trajectory_.position_W[2] << ","
            <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

        listDroneTrajectoryReference_.push_back(tempTrajectoryReferences.str());

     }
}

}
