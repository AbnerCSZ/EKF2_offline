#include "Myekf2.h"
#include <fstream>
class Ekf2;
std::ifstream read1("data/imu_data.txt");
namespace ekf2
{
Ekf2 *instance = nullptr;
}

Ekf2::Ekf2():
	_ekf(),
	_params(_ekf.getParamHandle())
{

}

Ekf2::~Ekf2()
{

}


void Ekf2::print_status()
{
	printf("local position OK %s", (_ekf.local_position_is_valid()) ? "[YES]" : "[NO]");
	printf("global position OK %s", (_ekf.global_position_is_valid()) ? "[YES]" : "[NO]");
}

void Ekf2::task_main()
{
	// initialise parameter cache// TODO
	//updateParams();
//	std::ifstream read1("data/imu_data.txt");

	double gyro_integral_dt = 0;
	double accelerometer_integral_dt = 0;
	double last_IMUtime = 0;
    double now = 0;

	while (!_task_should_exit) {

		bool isa = true;
		bool gps_updated = false;
		bool airspeed_updated = false;
		bool optical_flow_updated = false;
		bool range_finder_updated = false;
		bool vehicle_land_detected_updated = false;
		bool vision_position_updated = false;
		bool vision_attitude_updated = false;
		bool vehicle_status_updated = false;

		// long gyro_integral_dt = 0.01;
		// // in replay mode we are getting the actual timestamp from the sensor topic
			
		read1 >> now;
		printf("time now: %lf\n", now);
		gyro_integral_dt = now - last_IMUtime;
		accelerometer_integral_dt = now - last_IMUtime;

		// // push imu data into estimator
		double gyro_integral[3],gyro_rad[3];
		read1 >> gyro_rad[0];	read1 >> gyro_rad[1];	read1 >> gyro_rad[2];
		printf("gyro_rad:%lf,%lf,%lf\n", gyro_rad[0], gyro_rad[1], gyro_rad[2]);

		gyro_integral[0] = gyro_rad[0] * gyro_integral_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_integral_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_integral_dt;

		double accel_integral[3],accelerometer_m_s2[3];
		read1 >> accelerometer_m_s2[0];	read1 >> accelerometer_m_s2[1];	read1 >> accelerometer_m_s2[2];
		printf("accelerometer_m_s2:%lf,%lf,%lf\n", accelerometer_m_s2[0], accelerometer_m_s2[1], accelerometer_m_s2[2]);

		accel_integral[0] = accelerometer_m_s2[0] * accelerometer_integral_dt;
		accel_integral[1] = accelerometer_m_s2[1] * accelerometer_integral_dt;
		accel_integral[2] = accelerometer_m_s2[2] * accelerometer_integral_dt;
 
		//_ekf.setIMUData(now, gyro_integral_dt * 1.e6f, accelerometer_integral_dt * 1.e6f,
	//			gyro_integral, accel_integral);		

		// run the EKF update and output
		// if (_ekf.update()) {

		// 	matrix::Quaternion<float> q;
		// 	_ekf.copy_quaternion(q.data());

		// 	float velocity[3];
		// 	_ekf.get_velocity(velocity);

		// 	float gyro_rad[3];

		// 	{
		// 		// generate control state data
		// 		float gyro_bias[3] = {};
		// 		_ekf.get_gyro_bias(gyro_bias);
		// 		gyro_rad[0] = gyro_rad[0] - gyro_bias[0];
		// 		gyro_rad[1] = gyro_rad[1] - gyro_bias[1];
		// 		gyro_rad[2] = gyro_rad[2] - gyro_bias[2];

		// 		// Velocity in body frame
		// 		Vector3f v_n(velocity);
		// 		matrix::Dcm<float> R_to_body(q.inversed());
		// 		Vector3f v_b = R_to_body * v_n;


		// 		// Local Position NED
		// 		float position[3];
		// 		_ekf.get_position(position);
		// 		printf("%lf,%lf,%lf\n", position[0], position[1], position[2]);
		// 		// Attitude quaternion
		// 		//q.copyTo(ctrl_state.q);

		// 		//_ekf.get_quat_reset(&ctrl_state.delta_q_reset[0], &ctrl_state.quat_reset_counter);

		// 		// Acceleration data
		// 		matrix::Vector<float, 3> acceleration(accelerometer_m_s2);

		// 		float accel_bias[3];
		// 		_ekf.get_accel_bias(accel_bias);
		// 		// ctrl_state.x_acc = acceleration(0) - accel_bias[0];
		// 		// ctrl_state.y_acc = acceleration(1) - accel_bias[1];
		// 		// ctrl_state.z_acc = acceleration(2) - accel_bias[2];

		// 		// // compute lowpass filtered horizontal acceleration
		// 		acceleration = R_to_body.transpose() * acceleration;
		// 		// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
		// 		// 		acceleration(1) * acceleration(1));
		// 		// ctrl_state.horz_acc_mag = _acc_hor_filt;

		// 		// ctrl_state.airspeed_valid = false;

		// 	}
			
		// 	// generate vehicle local position data

		// 	float pos[3] = {};
		// 	// Position of body origin in local NED frame
		// 	_ekf.get_position(pos);

		// 	// Velocity of body origin in local NED frame (m/s)

		// 	// TODO: better status reporting
	

		// 	// Position of local NED origin in GPS / WGS84 frame
			
		// 	// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
		// 	//_ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		
		// 	// The rotation of the tangent plane vs. geographical north
		// 	matrix::Eulerf euler(q);
		
		
		// 	// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
		// 	Vector3f pos_var, vel_var;
		// 	_ekf.get_pos_var(pos_var);
		// 	_ekf.get_vel_var(vel_var);
	
		
		//} 

	}


}
int main(int argc, char *argv[])
{
	printf("asasssa\n" );
	Ekf2* _ekf2 = new Ekf2();
	//_ekf2->print_status();
	_ekf2->task_main();


	return 1;
}
