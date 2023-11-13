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
  void initAfterFailedRelease();

  void run() override;

protected:

  webots::Node* ned_robot_arm_;
  webots::Node* sphere_;
  webots::Node* cube_;
  int robot_time_step_;

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

private:

  enum State { STARTING, IDLE, MOVE_DOWN_CLOSE, MOVE_DOWN_OPEN, MOVE_DOWN_OPEN_FAIL, MOVE_UP, MOVE_ARM, CLOSE_GRIPPER, OPEN_GRIPPER, STOPPING };

  State state_;

  double ned_robot_arm_x_;
  double ned_robot_arm_y_;

  const double position_offset_ = -15.0;
  const double position_factor_ = 0.1;
  const double bin_size_ = 5;

  const double arm_up_ = 0.65;
  const double arm_down_ = 0.8;
  const double arm_down_fail_ = 0.72;
  const double jaw_open_ = 0.01;
  const double jaw_closed_ = -0.0019;

  const double sphere_rotation_[4] = { 0, 0, 1, 1.50014 };
  const double sphere_translation_[3] = { -0.3, -0.02, 0.01 };
  const double cube_rotation_[4] = { 0, 0, -1, 1.00257 };
  const double cube_translation_[3] = { -0.257, -0.161, 0.01 };

  const double position_accuracy_error_ = 0.001;
  const double gripper_accuracy_error_ = 0.0005;

  double target_h_position_;

  webots::Motor* joint_1_;
  webots::Motor* joint_2_;
  webots::Motor* joint_3_;
  webots::Motor* joint_4_;
  webots::Motor* joint_5_;
  webots::Motor* joint_6_;
  webots::Motor* joint_base_to_jaw_1_;
  webots::Motor* joint_base_to_jaw_2_;

  webots::PositionSensor* joint_1_sensor_;
  webots::PositionSensor* joint_2_sensor_;
  webots::PositionSensor* joint_3_sensor_;
  webots::PositionSensor* joint_5_sensor_;
  webots::PositionSensor* joint_6_sensor_;
  webots::PositionSensor* joint_base_to_jaw_1_sensor_;
  webots::PositionSensor* joint_base_to_jaw_2_sensor_;

  void executeCommand();

  double getPosition(const double* translation);

  double getAngularPosition(double angle);

};

