#include "hand_grab_sphere_controller.h"

HandGrabSphereController::HandGrabSphereController() : AERAController() {

  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  std::cout << "Timestep: " << robot_time_step_ << std::endl;
  sphere_ = ((webots::Supervisor*)robot_)->getFromDef("sphere");
  cube_ = ((webots::Supervisor*)robot_)->getFromDef("cube");

  joint_1_ = robot_->getMotor("joint_1");
  joint_2_ = robot_->getMotor("joint_2");
  joint_3_ = robot_->getMotor("joint_3");
  joint_5_ = robot_->getMotor("joint_5");
  joint_6_ = robot_->getMotor("joint_6");
  joint_base_to_jaw_1_ = robot_->getMotor("joint_base_to_jaw_1");
  joint_base_to_jaw_2_ = robot_->getMotor("joint_base_to_jaw_2");

  joint_1_sensor_ = robot_->getPositionSensor("joint_1_sensor");
  joint_base_to_jaw_1_sensor_ = robot_->getPositionSensor("joint_base_to_jaw_1_sensor");
  joint_base_to_jaw_2_sensor_ = robot_->getPositionSensor("joint_base_to_jaw_2_sensor");
}

HandGrabSphereController::~HandGrabSphereController() {
  if (robot_) {
    delete robot_;
  }
}

int HandGrabSphereController::start() {
  std::cout << "Starting HandGrabSphereController. Initializing entities, objects, and commands" << std::endl;
  std::vector<tcp_io_device::MetaData> objects;
  std::vector<tcp_io_device::MetaData> commands;
  fillIdStringMaps({ "hand", "cube", "sphere", "position", "grab", "release", "move" });
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["hand"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["cube"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["sphere"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));

  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["hand"], string_id_mapping_["grab"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["hand"], string_id_mapping_["release"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["hand"], string_id_mapping_["move"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));

  sendSetupMessage(objects, commands);
  waitForStartMsg();
  std::vector<tcp_io_device::MsgData> data_to_send;
  data_to_send.push_back(createMsgData<double>(objects[0], { 0.0 }));
  data_to_send.push_back(createMsgData<double>(objects[1], { 0.0 }));
  data_to_send.push_back(createMsgData<double>(objects[2], { 0.0 }));
  sendDataMessage(data_to_send);
  run();
}

void HandGrabSphereController::init() {
  // Decrease the PID gain from 10 so that we keep a grip on the object.
  joint_1_->setControlPID(9, 0, 0);
  // Increase the strength of the grip.
  joint_base_to_jaw_1_->setControlPID(250, 0, 0);
  joint_base_to_jaw_2_->setControlPID(250, 0, 0);

  joint_1_sensor_->enable(robot_time_step_);
  joint_base_to_jaw_1_sensor_->enable(robot_time_step_);
  joint_base_to_jaw_2_sensor_->enable(robot_time_step_);

  joint_2_->setPosition(arm_up_);
  joint_3_->setPosition(0.32);
  joint_5_->setPosition(-0.5);
  joint_6_->setPosition(3.1415296 / 2);

  joint_base_to_jaw_1_->setPosition(jaw_open_);
  joint_base_to_jaw_2_->setPosition(jaw_open_);
}

void HandGrabSphereController::run() {
  int aera_us = 0;
  diagnostic_mode_ = false;
  while (robot_->step(robot_time_step_) != -1) {
    if (!aera_started_) {
      std::cout << "AERA not started, wait for start message before calling run()" << std::endl;
      break;
    }
    auto msg = receive_queue_->dequeue();
    if (!msg && diagnostic_mode_)
    {
      while (!msg)
      {
        msg = receive_queue_->dequeue();
      }
    }
    if (msg) {
      if (msg->messagetype() == tcp_io_device::TCPMessage_Type_DATA) {
        handleDataMsg(dataMsgToMsgData(std::move(msg)));
      }
      else {
        std::cout << "Received message with unexpected type "
          << tcp_io_device::TCPConnection::type_to_name_map_[msg->messagetype()]
          << ". Ignoring the message..." << std::endl;
      }
    }
  }
  if (aera_us % 100'000) {

    double hand_position = joint_1_sensor_->getValue();
    const double* cube_position = cube_->getPosition();
    const double* sphere_position = sphere_->getPosition();
  }

  aera_us += robot_time_step_ * 100;
}

double absolutePositionToRelativeAngle(const double* absolute_position, const double* relative_view_point) {

}

void HandGrabSphereController::handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) {

}