#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <ekf.h>


{
	bool mag_updated = false;
	bool baro_updated = false;
	bool gps_updated = false;

	while(1)
	{
		//imu get
		float now = 0;	//s
		now *= 1.e6f;	//us
		//printf("time now: %lf\n", now);
		gyro_integral_dt = now - last_IMUtime;	//us
		gyro_integral_dt /= 1.e6f;	//s
		accelerometer_integral_dt = now - last_IMUtime;
		accelerometer_integral_dt /=1.e6f;

		// // push imu data into estimator
		float gyro_integral[3],gyro_rad[3];
		//printf("gyro:%lf,%lf,%lf,%lf s\n", gyro_rad[0], gyro_rad[1], gyro_rad[2],gyro_integral_dt);
		gyro_integral[0] = gyro_rad[0] * gyro_integral_dt;
		gyro_integral[1] = gyro_rad[1] * gyro_integral_dt;
		gyro_integral[2] = gyro_rad[2] * gyro_integral_dt;

		float accel_integral[3],accelerometer_m_s2[3];
		//printf("accelerometer_m_s2:%lf,%lf,%lf\n", accelerometer_m_s2[0], accelerometer_m_s2[1], accelerometer_m_s2[2]);
		accel_integral[0] = accelerometer_m_s2[0] * accelerometer_integral_dt;
		accel_integral[1] = accelerometer_m_s2[1] * accelerometer_integral_dt;
		accel_integral[2] = accelerometer_m_s2[2] * accelerometer_integral_dt;
		_ekf.setIMUData(now, gyro_integral_dt * 1.e6f, accelerometer_integral_dt * 1.e6f,
				gyro_integral, accel_integral);		
		last_IMUtime = now;


		if(mag_updated)
		{
			//get mag 
			magMutex.lock();
			_timestamp_mag_us = mag_from_API(0) * 1.e6f;
			magMutex.unlock();
			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the 50msec is reached.
			_mag_time_sum_ms += _timestamp_mag_us / 1000.0f;
			_mag_sample_count++;

			_mag_data_sum[0] += mag_from_API(1);
			_mag_data_sum[1] += mag_from_API(2);
			_mag_data_sum[2] += mag_from_API(3);
			magMutex.unlock();
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;
			
			if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv, _mag_data_sum[1] *mag_sample_count_inv, _mag_data_sum[2] *mag_sample_count_inv};
				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
				printf("mag: %f %f %d %f %f %f\n",now,mag_time_s_read,mag_time_ms,
						_mag_data_sum[0],_mag_data_sum[1],_mag_data_sum[2]);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;	
			}	
			mag_updated = false;	
		}

		if(baro_updated)
		{
				baroMutex.lock();
				_timestamp_balt_us = baro_from_API(0) *1.e6f;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the 50msec is reached.
				_balt_time_sum_ms += _timestamp_balt_us / 1000;
				_balt_sample_count++;
				_balt_data_sum += baro_from_API(1);
				baroMutex.unlock();
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
				float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
				printf("baro: %f %f %d %f\n",now,baro_time_s_read,balt_time_ms,
						balt_data_avg);					
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;

				}		
				baro_updated = false;	
		}

		if(gps_updated)
		{
			struct gps_message gps_msg = {};
			gpsMutex.lock();
			gps_msg.time_usec = (uint64_t)(gps_from_API(0) * 1.e6f);
			gps_msg.lat = (int32_t)(gps_from_API(1) * 1.e7f);
			gps_msg.lon = (int32_t)(gps_from_API(2) * 1.e7f);
			gps_msg.alt = (int32_t)(gps_from_API(2) * 1.e3f);
			gpsMutex.unlock();
			printf("time now: %lf\n", now);
			printf("gps: %ld, %d, %d, %d\n",gps_msg.time_usec,gps_msg.lat,gps_msg.lon,gps_msg.alt);
			gps_msg.fix_type = 3;
			gps_msg.eph = 0.3;
			gps_msg.epv = 0.4;
			gps_msg.sacc = 0;
			gps_msg.vel_m_s = 0;
			gps_msg.vel_ned[0] = 110;
			gps_msg.vel_ned[1] = 110;
			gps_msg.vel_ned[2] = 10 1;

			gps_msg.vel_ned_valid = 1;
			gps_msg.nsats = 8;
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps_msg.time_usec, &gps_msg);
			gps_updated = flase;
		}


		if (_ekf.update()) {
			printf("zyxloveljs\n");

			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);

			float gyro_rad[3];

			{
				// generate control state data
				float gyro_bias[3] = {};
				_ekf.get_gyro_bias(gyro_bias);
				gyro_rad[0] = gyro_rad[0] - gyro_bias[0];
				gyro_rad[1] = gyro_rad[1] - gyro_bias[1];
				gyro_rad[2] = gyro_rad[2] - gyro_bias[2];

				// Velocity in body frame
				Vector3f v_n(velocity);
				matrix::Dcm<float> R_to_body(q.inversed());
				Vector3f v_b = R_to_body * v_n;


				// Local Position NED
				float position[3];
				_ekf.get_position(position);
				printf("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);
				position_estimator<< now/1.e6f <<" "<<position[0] <<" "<<position[1] <<" "
				<<position[2] <<" "<<std::endl;
				// Attitude quaternion
				//q.copyTo(ctrl_state.q);

				//_ekf.get_quat_reset(&ctrl_state.delta_q_reset[0], &ctrl_state.quat_reset_counter);

				// Acceleration data
				matrix::Vector<float, 3> acceleration(accelerometer_m_s2);

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				// ctrl_state.x_acc = acceleration(0) - accel_bias[0];
				// ctrl_state.y_acc = acceleration(1) - accel_bias[1];
				// ctrl_state.z_acc = acceleration(2) - accel_bias[2];

				// // compute lowpass filtered horizontal acceleration
				acceleration = R_to_body.transpose() * acceleration;
				// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
				// 		acceleration(1) * acceleration(1));
				// ctrl_state.horz_acc_mag = _acc_hor_filt;

				// ctrl_state.airspeed_valid = false;

			}
			
			// generate vehicle local position data

			float pos[3] = {};
			// Position of body origin in local NED frame
			_ekf.get_position(pos);
			//printf("%f  %f  %f\n", pos[0],pos[1],pos[2]);

			// Velocity of body origin in local NED frame (m/s)

			// TODO: better status reporting
	

			// Position of local NED origin in GPS / WGS84 frame
			
			// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
			//_ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		
			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			printf("euler: %f  %f  %f\n", euler.phi(),euler.theta(),euler.psi());
				euler_estimator<< now/1.e6f <<" "<<euler.phi() <<" "<<euler.theta() <<" "
				<<euler.psi() <<" "<<std::endl;			
			// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
			Vector3f pos_var, vel_var;
			_ekf.get_pos_var(pos_var);
			_ekf.get_vel_var(vel_var);
		}
	}
}

{
	magMutex.lock();
	mag_from_API << timestamp , , ,     ;
	mag_updated = true;
	magMutex.unlock();
}

{
	baroMutex.lock();
	baro_from_API << timestamp,  ;
	baro_updated = true;
	baroMutex.unlock();
}

{
	gpsMutex.lock();
	gps_from_API << timestamp, , , ;
	gps_updated = true;
	gpsMutex.unlock();
}

int main(int argc, char *argv[])
{
	
	Ekf* _ekf = new Ekf();


	return 0;
}