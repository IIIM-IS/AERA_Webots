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

  webots::Node* sphere_;
  webots::Node* cube_;
  int robot_time_step_;

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

private:

  const double arm_up_ = 0.65;
  const double arm_down_ = 0.8;
  const double jaw_open_ = 0.01;
  const double jaw_closed_ = -0.0019;

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

};

