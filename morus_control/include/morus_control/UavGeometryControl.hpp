/*
 * UavGeometricControl.h
 *
 *	UavGeometricControl class implements geometric control algorithm for
 *	moving-mass UAVs.
 *	Geometric control will handle both height and attitude control
 *	simultaneously.
 *
 *  Created on: May 10, 2018
 *      Author: lmark
 */

#ifndef UAV_GEOMETRY_CONTROL_H
#define UAV_GEOMETRY_CONTROL_H

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <eigen3/Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <mmuav_control/UavGeometryControlParamsConfig.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <mmuav_msgs/GeomCtlStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace Eigen;

class UavGeometryControl
{

	public:

		/**
		 * Class constructor.
		 *
		 * @param rate - Controller rate.
		 * @param uav_ns - uav namespace
		 */
		UavGeometryControl(int rate, std::string uav_ns);

		/**
		 * Class destructor.
		 */
		virtual ~UavGeometryControl();

		/**
		 * Runs the geometric control algorithm until ROS shuts down.
		 */
		void runControllerLoop();

		void enableMassControl();
		void disableMassControl();

	private:

		void imu_cb(const sensor_msgs::Imu &msg);
		void pose_cb(const geometry_msgs::PoseStamped &msg);
		void vel_cb(const geometry_msgs::TwistStamped &msg);
		void xd_cb(const geometry_msgs::Vector3 &msg);
		void vd_cb(const geometry_msgs::Vector3 &msg);
		void ad_cb(const geometry_msgs::Vector3 &msg);
		void omegad_cb(const geometry_msgs::Vector3 &msg);
		void alphad_cb(const geometry_msgs::Vector3 &msg);
		void b1d_cb(const geometry_msgs::Vector3 &msg);
		void rd_cb(const std_msgs::Float64MultiArray &msg);
		void euler_cb(const geometry_msgs::Vector3 &msg);
		void ctl_mode_cb(const std_msgs::Int8 &msg);
		void param_cb(
				mmuav_control::UavGeometryControlParamsConfig &config,
				uint32_t level);

		/**
		 * This method will keep running until all sensors
		 * have returned callbacks.
		 */
		void sensorChecks();

		/**
		 * Calculate b3_d and f_u as position tracking control inputs.
		 *
		 * @param pos_desired - desired position reference
		 * 						(may change due to prefilter)
		 * @param pos_old - old position
		 * @param dt - time interval
		 * @param b3_d - thrust heading reference, assigned in method
		 * @param f_u - thrust magnitude value, assigned in method
		 */
		void trajectoryTracking(
				const Matrix<double, 3, 1> pos_desired,
				const Matrix<double, 3, 1> pos_old,
				const double dt,
				Matrix<double, 3, 1> &b3_d,
				double &f_u);

		/**
		 * Calculate control moments M_u used for attitude tracking.
		 *
		 * @param b1_d - desired heading
		 * @param b3_d - desired thrust vector
		 * @param dt - time interval
		 * @param R_c_old - reference for old calculated rotation matrix
		 * 					(Position tracking)
		 * @param omega_c_old - reference for old calculated angular velocity
		 * 						(Position tracking)
		 * @param M_u - control moments, assigned in method
		 */
		void attitudeTracking(
				const Matrix<double, 3, 1> b1_desired,
				const Matrix<double, 3, 1> b3_desired,
				const double dt,
				Matrix<double, 3, 3> &R_c_old,
				Matrix<double, 3, 3> &omega_c_old,
				Matrix<double, 3, 1> &M_u);

		/**
		 * Calculate and set desired angular velocity and
		 * desired angular acceleration based on R_c[k] and R_c[k-1].
		 *
		 * @param R_c - Calculated R
		 */
		void calculateDesiredAngularVelAndAcc(
				const Matrix<double, 3, 3> R_c);

		/**
		 * Calculate rotor velocities from given vector containing
		 * total thrust and moments M_x, M_y, M_z.
		 *
		 * @param thrust_moment_vec
		 * @param transform_matrix - matrix transforms given vector to forces
		 * @rotor_velocities
		 */
		void calculateRotorVelocities(
				Matrix<double, 4, 1> thrust_moment_vec,
				Matrix<double, 4, 4> transform_matrix,
				Matrix<double, 4, 1>& rotor_velocities);

		/**
		 * Perform quaternion to euler transformation.
		 *
		 * @param quaternion: 4 dimensional float array.
		 * @param euler: 3 dimensional float array.
		 */
		void quaternion2euler(float *quaternion, float *euler);

		/**
		 * Perform hat operator on given vector components.
		 *
		 * @param x - x vector component
		 * @param y - y vector component
		 * @param z - z vector component
		 * @param hatMatrixs - Matrix of the following form:
		 * 	[ 0  -z,  y]
		 * 	[ z,  0, -x]
		 * 	[-y,  x,  0]
		 */
		void hatOperator(
				const double x,
				const double y,
				const double z,
				Matrix<double, 3, 3> &hatMatrix);

		/**
		 * Perform a vee( V ) operator on a given hatMatrix.
		 * It is implied that the given hatMatrix is a skew matrix.
		 * It will decompose the skew matrix into a given veeVector reference.
		 *
		 * @param hatMatrx
		 * @param veeVector
		 */
		void veeOperator(
				const Matrix<double, 3, 3> hatMatrix,
				Matrix<double, 3, 1> &veeVector);
		/**
		 * Euler angles represented as a rotation matrix.
		 *
		 * @param roll
		 * @param pitch
		 * @param yaw
		 * @param rotMatrix - Rotation matrix will be stored here
		 */
		void euler2RotationMatrix(
				const double roll,
				const double pitch,
				const double yaw,
				Matrix<double, 3, 3> &rotMatrix);

		/**
		 * Perform saturation filter on the given value;
		 *
		 * @param value
		 * @param lowLimit
		 * @param highLimit
		 *
		 * @return saturated value
		 */
		double saturation(
				double value,
				double lowLimit,
				double highLimit);

		/**
		 * Perform deadzone filter on given value.
		 *
		 * @param value
		 * @param lowLimit
		 * @param highLimit
		 */
		double deadzone(
				double value,
				double lowLimit,
				double highLimit);

		/**
		 * Node handle used for setting up subscribers and publishers.
		 */
		ros::NodeHandle node_handle_;

		/**
		 * Subscriber handle for the IMU, position
		 * and (relative) velocity topics.
		 */
		ros::Subscriber imu_ros_sub_, pose_ros_sub_, velocity_ros_sub_;

		/**
		 * - Motor velocities publisher
		 * - Attitude error publisher
		 * - Mass 0 command publisher
		 * - Mass 1 command publisher
		 * - Mass 2 command publisher
		 * - Mass 3 command publisher
		 */
		ros::Publisher rotor_ros_pub_, status_ros_pub_, mass0_cmd_pub_,
					   mass1_cmd_pub_, mass2_cmd_pub_, mass3_cmd_pub_;

		/**
		 * Subscriber handle for:
		 * - desired position reference
		 * - desired linear velocity reference
		 * - desired linear acceleration reference
		 * - desired heading reference
		 * - desired angular velocity
		 * - desired angular acceleration
		 * - desired control mode ( position / attitude )
		 * - orientation: roll, pitch, yaw
		 */
		ros::Subscriber xd_ros_sub_, vd_ros_sub_, ad_ros_sub_,
						b1d_ros_sub_, omega_d_ros_sub_, rd_ros_sub_,
						alpha_d_ros_sub_, ctl_mode_ros_sub_,
						euler_ros_sub_;

		/**
		 * Messages containing angle measured values and
		 * angle rate measured values respectively.
		 */
		geometry_msgs::Vector3 euler_mv_, euler_rate_mv_;

		/**
		 * CONTROLLER INPUT REFERENCES - position
		 * 	- desired position x_d_
		 * 	- desired linear velocity v_d_
		 * 	- desired linear acceleration a_d_
		 */
		Matrix<double, 3, 1> x_d_, v_d_, a_d_;
		Matrix<double, 3, 1>  x_mv_, v_mv_, a_mv_;

		/**
		 * 	CONTROLLER INPUT REFERENCES - attitude
		 * 	- desired angular velocity omega_d_
		 * 	- desired angular acceleration alpha_d_
		 * 	- desired direction of the first body axis b1_D
		 * 	- desired euler angles (as a means to set R_d)
		 */
		Matrix<double, 3, 1> omega_d_, alpha_d_;
		Matrix<double, 3, 1> omega_mv_, alpha_mv_;
		Matrix<double, 3, 1> b1_d_, euler_d_;

		/**
		 * R_mv_ - Measured rotation matrix.
		 * R_d_  - Desired rotation matrix
		 */
		Matrix<double, 3, 3> R_mv_, R_d_;

		/**
		 * CONTROLLER PARAMETERS:
		 *	- k_x: position tracking error gain (eX)
		 *	- k_v: velocity tracking error gain (eV)
		 *	- k_R: orientation matrix error gain (eR)
		 *	- k_omega: angular velocity error gain (eOmega)
		 */
		Matrix<double, 3, 3> k_x_, k_v_, k_R_, k_omega_;

		/**
		 * Controller status message.
		 */
		mmuav_msgs::GeomCtlStatus status_msg_;

		/**
		 * Variable used for calculating time intervals in the controller loop.
		 */
		ros::Time t_old_;

		/**
		 * Controller rate. Frequency at which the loop in run method
		 * will be executed.
		 */
		int controller_rate_;

		/**
		 * True if mass control is enabled, otherwise false.
		 */
		bool enable_mass_control_;

		/**
		 * Sleep duration used while performing checks before starting the
		 * run() loop.
		 */
		float sleep_duration_;

		/**
		 * True when imu callback function occurred otherwise false.
		 */
		bool imu_start_flag_;

		/**
		 * True when pose callback function occured otherwise false.
		 */
		bool pose_start_flag_;

		/**
		 * True when velocity callback function occured otherwise false.
		 */
		bool velocity_start_flag_;

		/**
		 * True when param callback occured, otherwise false.
		 */
		bool param_start_flag_;

		/**
		 * Current control mode:
		 * 	- position control
		 * 	- attitude control
		 * 	- linear velocity control
		 */
		int current_control_mode_;

		bool calc_desired;
		Matrix<double, 3, 3> R_c_old, R_c_dot_old, omega_c_old;
		Matrix<double, 3, 1> x_old, x_dot_old;

		/**
		 * Dynamic reconfigure server.
		 */
		dynamic_reconfigure::
			Server<mmuav_control::UavGeometryControlParamsConfig> server_;

		/**
		 * Reconfigure callback for setting parameters.
		 */
		dynamic_reconfigure::
			Server<mmuav_control::UavGeometryControlParamsConfig>::
			CallbackType param_callback_;
};

#endif /* UAV_GEOMETRY_CONTROL_H */
