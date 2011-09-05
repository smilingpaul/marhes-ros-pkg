#include "odom_ekf/ins.h"

// state: px, py, pz, vx, vy, vz, yaw, pitch, roll
Ins::Ins():state_(NUM_STATES_), state_last_(NUM_STATES_), 
           state_last2_(NUM_STATES_), Cnb_(3, 3), fn_(3), fbias_(3), wbias_(3), g_(3)
{
  state_ = 0;
  state_last_ = 0;
  state_last2_ = 0;
  fbias_ = 0;
  wbias_ = 0;
  g_(1) = 0; g_(2) = 0; g_(3) = 9.80665;
  initialized_ = false;
  CalcDCM(state_(7), state_(8), state_(9));
}

bool Ins::Initialize(sensor_msgs::Imu msg)
{
  static int imu_rx_cnt = 0;
  static ColumnVector ft(3);
  btQuaternion q;
  double rx, ry, rz;
  
  if (imu_rx_cnt < 1)
  {
    ft = 0;
	  tf::quaternionMsgToTF(msg.orientation, q);
	  btMatrix3x3(q).getRPY(state_(9), state_(8), state_(7));
  	
  	fn_(1) = msg.linear_acceleration.x;
  	fn_(2) = msg.linear_acceleration.y;
  	fn_(3) = msg.linear_acceleration.z;
  	
  	wbias_(1) = msg.angular_velocity.x;
  	wbias_(2) = msg.angular_velocity.y;
  	wbias_(3) = msg.angular_velocity.z;
	}
	else if (imu_rx_cnt > 500)
	{
	  ft(1) = -g_(3) * sin(state_(8));
  	ft(2) = g_(3) * sin(state_(9)) * cos(state_(8));
  	ft(3) = g_(3) * cos(state_(9)) * cos(state_(8));

	  fbias_(1) = fn_(1) - ft(1);
	  fbias_(2) = fn_(2) - ft(2);
	  fbias_(3) = fn_(3) - ft(3);
	  
	  CalcDCM(state_(7), state_(8), state_(9));
	  state_last2_ = state_;
	  state_last_ = state_;	  
	  
	  ROS_INFO("Average Force: %f, %f, %f", fn_(1), fn_(2), fn_(3));
	  ROS_INFO("Expected Force: %f, %f, %f", ft(1), ft(2), ft(3));
	  ROS_INFO("Bias Force: %f, %f, %f", fbias_(1), fbias_(2), fbias_(3));
	  ROS_INFO("Bias AngVel: %f, %f, %f", wbias_(1), wbias_(2), wbias_(3));
	  ROS_INFO("Angles: %f, %f, %f", state_(7), state_(8), state_(9));
	  
	  initialized_ = true;
	}
  else
  {
  	tf::quaternionMsgToTF(msg.orientation, q);
	  btMatrix3x3(q).getRPY(rx, ry, rz);
    
    state_(7) = (state_(7) + rz) / 2;
    state_(8) = (state_(8) + ry) / 2;
    state_(9) = (state_(9) + rx) / 2;
    
  	fn_(1) = (fn_(1) + msg.linear_acceleration.x) / 2;
  	fn_(2) = (fn_(2) + msg.linear_acceleration.y) / 2;
  	fn_(3) = (fn_(3) + msg.linear_acceleration.z) / 2;
  	
  	wbias_(1) = (wbias_(1) + msg.angular_velocity.x) / 2;
  	wbias_(2) = (wbias_(2) + msg.angular_velocity.y) / 2;
  	wbias_(3) = (wbias_(3) + msg.angular_velocity.z) / 2;
  }	
	
	imu_msg_last_ = msg;
  imu_rx_cnt++;
  
  return initialized_;
}

void Ins::Integrate(sensor_msgs::Imu msg)
{
  if (initialized_)
  {
    // Get elapsed time to do integration
    dt_ = (msg.header.stamp - imu_msg_last_.header.stamp).toSec();
    
    // Update states for simpson's rule calculation
    state_last2_ = state_last_;
    state_last_ = state_;
  
    // Get the accelerations
    ColumnVector fb(3), angle_change(3), wb(3);
    fb(1) = msg.linear_acceleration.x;
    fb(2) = msg.linear_acceleration.y;
    fb(3) = msg.linear_acceleration.z;
        
    // Get the angular accelerations
    wb(1) = msg.angular_velocity.x;
    wb(2) = msg.angular_velocity.y;
    wb(3) = msg.angular_velocity.z;
    
    //wb -= wbias_;
    
    // Update Cnb
    angle_change(1) = wb(1) * dt_;
    angle_change(2) = wb(2) * dt_;
    angle_change(3) = wb(3) * dt_;
    double angle_change_mag = sqrt(angle_change(1) * angle_change(1) + 
                                   angle_change(2) * angle_change(2) + 
                                   angle_change(3) * angle_change(3));
    double alpha = sin(angle_change_mag) / angle_change_mag;
    double beta = (1 - cos(angle_change_mag)) / (angle_change_mag * angle_change_mag);
    Matrix angle_skew(3, 3);
    angle_skew(1, 1) = 0; angle_skew(1, 2) = -angle_change(3); angle_skew(1, 3) = angle_change(2);
    angle_skew(2, 1) = angle_change(3); angle_skew(2, 2) = 0; angle_skew(2, 3) = -angle_change(1);
    angle_skew(3, 1) = -angle_change(2); angle_skew(3, 2) = angle_change(1); angle_skew(3, 3) = 0;    
    Matrix I(3, 3);
    I = 0; I(1, 1) = 1; I(2, 2) = 1; I(3, 3) = 1;
    Cnb_ = Cnb_ * (I + angle_skew * alpha + angle_skew * angle_skew * beta);
    
    // Fix Cnb
    CorrectDCM();
    
    //ROS_INFO("A: %f, %f, %f", fb(1) - fbias_(1), fb(2) - fbias_(2), fb(3) - fbias_(3));
     
    // Get the accelerations in the N frame
    fn_ = (Cnb_ * (fb - fbias_)) - g_;
    //fn_ = 0;
    
    //ROS_INFO("FN: %f, %f, %f", fn_(1), fn_(2), fn_(3));
    
    // Update the euler angles from the angular velocities
    state_(7) += ((wb(2) * sin(state_(9)) + wb(3) * cos(state_(9))) / cos(state_(8))) * dt_;
    state_(8) += (wb(2) * cos(state_(9)) - wb(3) * sin(state_(9))) * dt_;
    state_(9) += ((wb(2) * sin(state_(9)) + wb(3) * cos(state_(9))) * tan(state_(8)) + wb(1)) * dt_;
      
    // Update the velocities from the forces, fn
    state_(4) += fn_(1) * dt_;
    state_(5) += fn_(2) * dt_;
    state_(6) += fn_(3) * dt_;
    
    // Integrate the velocities to positions
    state_(1) += (state_last2_(4) + 4 * state_last_(4) + state_(4)) / 3 * dt_;
    state_(2) += (state_last2_(5) + 4 * state_last_(5) + state_(5)) / 3 * dt_;
    state_(3) += (state_last2_(6) + 4 * state_last_(6) + state_(6)) / 3 * dt_;
  }
  
  imu_msg_last_ = msg;
  
}

ColumnVector Ins::GetState()
{
  return state_;
}

ColumnVector Ins::GetLastFn()
{
  return fn_;
}

Matrix Ins::GetCnb()
{
  return Cnb_;
}

double Ins::GetDt()
{
  return dt_;
}

void Ins::Correct(ColumnVector errors)
{
  state_ -= errors;
}

RowVector Ins::CrossProduct(RowVector a, RowVector b)
{
  RowVector ans(3);
  ans(1) = a(2) * b(3) - a(3) * b(2);
  ans(2) = a(1) * b(3) - a(3) * b(1);
  ans(3) = a(1) * b(2) - a(2) * b(1);
  
  return ans;
}

RowVector Ins::Normalize(RowVector a)
{
  double l = sqrt(a(1) * a(1) + a(2) * a(2) + a(3) * a(3));
  return (a / l);
}

void Ins::CorrectDCM()
{
  double angErr = Cnb_.rowCopy(1) * Cnb_.rowCopy(2).transpose() * 0.5;
  Cnb_(1, 1) -= angErr * Cnb_(2, 1);
  Cnb_(1, 2) -= angErr * Cnb_(2, 2);
  Cnb_(1, 3) -= angErr * Cnb_(2, 3);
  Cnb_(2, 1) -= angErr * Cnb_(1, 1);
  Cnb_(2, 2) -= angErr * Cnb_(1, 2);
  Cnb_(2, 3) -= angErr * Cnb_(1, 3);
  
  RowVector z = CrossProduct(Cnb_.rowCopy(1), Cnb_.rowCopy(2));
  z = Normalize(z);
  Cnb_(3, 1) = z(1); Cnb_(3, 2) = z(2); Cnb_(3, 3) = z(3);
  
  z = Normalize(Cnb_.rowCopy(1));
  Cnb_(1, 1) = z(1); Cnb_(1, 2) = z(2); Cnb_(1, 3) = z(3);
  z = Normalize(Cnb_.rowCopy(2));
  Cnb_(2, 1) = z(1); Cnb_(2, 2) = z(2); Cnb_(2, 3) = z(3);
}

void Ins::CalcDCM(double yaw, double pitch, double roll)
{
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);

  Cnb_(1,1) = cp * cy;
  Cnb_(1,2) = cr * sy + sr * sp * cy;
  Cnb_(1,3) = sr * sy - cr * sp * cy;
  Cnb_(2,1) = -cp * sy;
  Cnb_(2,2) = cr * cy - sr * sp * sy;
  Cnb_(2,3) = sr * cy + cr * sp * sr;
  Cnb_(3,1) = sp;
  Cnb_(3,2) = -sr * cp;
  Cnb_(3,3) = cr * cp; 
  /*
  Cnb_(1,1) = cp * cy;
  Cnb_(1,2) = -cr * sy + sr * sp * cy;
  Cnb_(1,3) = sr * sy + cr * sp * cy;
  Cnb_(2,1) = cp * sy;
  Cnb_(2,2) = cr * cy + sr * sp * sy;
  Cnb_(2,3) = -sr * cy + cr * sp * sr;
  Cnb_(3,1) = -sp;
  Cnb_(3,2) = sr * cp;
  Cnb_(3,3) = cr * cp;  
  */
}
