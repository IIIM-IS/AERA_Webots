#pragma once
#include "aera_controller.h"
class HandGrabSphereController :
  public AERAController
{
public:
  HandGrabSphereController();
  ~HandGrabSphereController();

  int start() override;

  void init();

  void run() override;

protected:

  webots::Node* ned_robot_arm_;
  webots::Node* sphere_;
  webots::Node* cube_;
  int robot_time_step_;

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

private:

  double ned_robot_arm_x_;
  double ned_robot_arm_y_;

  const double position_offset_ = -15.0;
  const double position_factor_ = 0.1;
  const double bin_size_ = 5;

  const double arm_up_ = 0.65;
  const double arm_down_ = 0.8;
  const double jaw_open_ = 0.01;
  const double jaw_closed_ = -0.0019;

  const double sphere_rotation_[4] = { 0, 0, 1, 1.50014 };
  const double sphere_translation_[3] = { -0.3, -0.025, 0.01 };
  const double cube_rotation_[4] = { 0, 0, -1, 1.00257 };
  const double cube_translation_[3] = { -0.257, -0.161, 0.01 };

  webots::Motor* joint_1_;
  webots::Motor* joint_2_;
  webots::Motor* joint_3_;
  webots::Motor* joint_4_;
  webots::Motor* joint_5_;
  webots::Motor* joint_6_;
  webots::Motor* joint_base_to_jaw_1_;
  webots::Motor* joint_base_to_jaw_2_;

  webots::PositionSensor* joint_1_sensor_;
  webots::PositionSensor* joint_base_to_jaw_1_sensor_;
  webots::PositionSensor* joint_base_to_jaw_2_sensor_;

  double getPosition(const double* translation);

  double getAngularPosition(double angle);

};

