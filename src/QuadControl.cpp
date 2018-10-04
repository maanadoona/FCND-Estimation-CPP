#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
  //printf("%f, %f, %f, %f, %f, %f, %f, [%f, %f, %f]\n", kpPosXY, kpPosZ, KiPosZ, kpVelXY, kpVelZ, kpBank, kpYaw, kpPQR.x, kpPQR.y, kpPQR.z);

#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

/*
    T = F * dperp
    [1  1  1  1]     [w1^2]     [c_bar]
    [1 -1 -1  1]     [w2^2]     [p_bar]
    [1  1 -1 -1]  x  [w3^2]  =  [q_bar]
    [1 -1  1 -1]     [w4^2]     [r_bar]

    w1_square = (c_bar + p_bar + q_bar + r_bar) / 4
    w2_square = (c_bar - p_bar + q_bar - r_bar) / 4
    w3_square = (c_bar - p_bar - q_bar + r_bar) / 4
    w4_square = (c_bar + p_bar - q_bar - r_bar) / 4
     FL    |    FR
    f1(0)  |   f2(1)
    -----------------
    f4(2)  |   f3(3)
     RL    |    RR
*/
    float l = L / sqrtf(2.f);
    float c_bar = collThrustCmd;
    float p_bar = momentCmd.x / l;
    float q_bar = momentCmd.y / l;
    float r_bar = -momentCmd.z / kappa;

    cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;   // f1 - FL
    cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f;   // f2 - FR
    cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) / 4.f;   // f3 - RR
    cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f;   // f4 - RL
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // p_c = pqrCmd.x
    // q_c = pqrCmd.y
    // r_c = pqrCmd.z
    // p_actual = pqr.x
    // q_actual = pqr.y
    // r_actual = pqr.z
/*
    p_err = p_c - p_actual
    q_err = q_c - q_actual
    r_err = r_c - r_actual
    u_bar_p = self.k_p_p * p_err
    u_bar_q = self.k_p_q * q_err
    u_bar_r = self.k_p_r * r_err
*/
    momentCmd.x = Ixx * kpPQR.x * (pqrCmd.x - pqr.x);
    momentCmd.y = Iyy * kpPQR.y * (pqrCmd.y - pqr.y);
    momentCmd.z = Izz * kpPQR.z * (pqrCmd.z - pqr.z);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // b_x_c_target = accelCmd.x
    // b_y_c_target = accelCmd.y
    // rot_mat = R( , )
/*
    b_dot_x_c = self.k_p_roll*(b_x_c_target - rot_mat[0, 2])
    b_dot_y_c = self.k_p_pitch*(b_y_c_target - rot_mat[1, 2])
    p_c = (rot_mat[1, 0]*b_dot_x_c - rot_mat[0, 0]*b_dot_y_c)/rot_mat[2, 2]
    q_c = (rot_mat[1, 1]*b_dot_x_c - rot_mat[0, 1]*b_dot_y_c)/rot_mat[2, 2]
*/
  pqrCmd.x = 0;
  pqrCmd.y = 0;
  pqrCmd.z = 0;

  if ( collThrustCmd > 0 )
  {
      float c = - collThrustCmd / mass;
      float b_x_cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
      float b_x_err = b_x_cmd - R(0,2);
      float b_x_p_term = kpBank * b_x_err;

      float b_y_cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
      float b_y_err = b_y_cmd - R(1,2);
      float b_y_p_term = kpBank * b_y_err;

      pqrCmd.x = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
      pqrCmd.y = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////    

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // z_target = posZCmd
  // z_dot_target = velZCmd
  // z_dot_dot_target = accelZCmd
  // z_actual = posZ
  // z_dot_actual = velZ
  // rot_mat = R(2, 2)
  // z_k_p = kpPosZ
  // z_k_d = kpVelZ
  // g = 9.81f

  //u_bar1 = self.z_k_p * (z_target - z_actual) + self.z_k_d * (z_dot_target-z_dot_actual) + z_dot_dot_target
  //c = (u_bar1 - self.g) / rot_mat[2,2]
  float z_err = posZCmd - posZ;
  float z_dot_err = velZCmd - velZ;
  integratedAltitudeError += z_err * dt;

  float u_bar1 = kpPosZ * z_err + kpVelZ * z_dot_err + KiPosZ * (integratedAltitudeError) + accelZCmd;
  //float u_bar1 = kpPosZ * z_err + kpVelZ * z_dot_err + KiPosZ * (integratedAltitudeError);
  thrust = -mass * (u_bar1 - 9.81f) / R(2,2);

  if(thrust > 10)
      thrust = 10;
  else if (thrust < 0.0)
      thrust = 0.0;

  //thrust = -mass * CONSTRAIN(thrust, -maxAscentRate / dt, maxDescentRate / dt);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // x_target = posCmd.x
    // x_dot_target = velCmd.x
    // x_dot_dot_target = accelCmdFF.x
    // x_actual = pos.x
    // x_dot_actual = vel.x
    // y_target = posCmd.y
    // y_dot_target = velCmd.y
    // y_dot_dot_target = accelCmdFF.y
    // y_actual = pos.y
    // y_dot_actual = vel.y
  /*
  termx1 = self.x_k_p * (x_target - x_actual)
  termx2 = self.x_k_d * (x_dot_target - x_dot_actual)
  x_dot_dot_cmd = termx1 + termx2 + x_dot_dot_target
  termy1 = self.y_k_p * (y_target - y_actual)
  termy2 = self.y_k_d * (y_dot_target - y_dot_actual)
  y_dot_dot_cmd = termy1 + termy2 + y_dot_dot_target
  */
    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
    velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

    float termx1 = kpPosXY * (posCmd.x - pos.x);
    float termx2 = kpVelXY * (velCmd.x - vel.x);
    accelCmd.x = termx1 + termx2 + accelCmd.x;
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    float termy1 = kpPosXY * (posCmd.y - pos.y);
    float termy2 = kpVelXY * (velCmd.y - vel.y);
    accelCmd.y = termy1 + termy2 + accelCmd.y;
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  /////////////////////////////// END STUDENT CODE ////////////////////////////
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float yaw_cmd_2_pi = 0;
  if ( yawCmd > 0 ) {
    yaw_cmd_2_pi = fmodf(yawCmd, 2 * F_PI);
  } else {
    yaw_cmd_2_pi = -fmodf(-yawCmd, 2 * F_PI);
  }
  float err = yaw_cmd_2_pi - yaw;
  if ( err > F_PI ) {
    err -= 2 * F_PI;
  } if ( err < -F_PI ) {
    err += 2 * F_PI;
  }
  yawRateCmd = kpYaw * err;
    //float err = yawCmd - yaw;
    //yawRateCmd = kpYaw * err;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
