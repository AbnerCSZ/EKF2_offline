#include "Myekf2.h"
#include <fstream>
class Ekf2;

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
	std::ifstream read1("data/imu_data.txt");

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
		// long now = 0;
		// now+=gyro_integral_dt;
		// //now = sensors.timestamp;

		// // push imu data into estimator
		// float gyro_integral[3];
		// gyro_integral[0] = sensors.gyro_rad[0] * gyro_integral_dt;
		// gyro_integral[1] = sensors.gyro_rad[1] * gyro_integral_dt;
		// gyro_integral[2] = sensors.gyro_rad[2] * gyro_integral_dt;
		// float accel_integral[3];
		// accel_integral[0] = sensors.accelerometer_m_s2[0] * sensors.accelerometer_integral_dt;
		// accel_integral[1] = sensors.accelerometer_m_s2[1] * sensors.accelerometer_integral_dt;
		// accel_integral[2] = sensors.accelerometer_m_s2[2] * sensors.accelerometer_integral_dt;
		// _ekf.setIMUData(now, sensors.gyro_integral_dt * 1.e6f, sensors.accelerometer_integral_dt * 1.e6f,
		// 		gyro_integral, accel_integral);		


	}


}
int main(int argc, char *argv[])
{
	printf("asasssa\n" );
	Ekf2* _ekf2 = new Ekf2();
	_ekf2->print_status();
	_ekf2->task_main();


	return 1;
}
