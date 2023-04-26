#include "hand_grab_sphere_controller.h"

#define DEBUG 1

HandGrabSphereController::HandGrabSphereController() : AERAController() {

  state_ = STARTING;

  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  std::cout << "Timestep: " << robot_time_step_ << std::endl;

  std::cout << "Getting Sphere, Cube, and Ned from WorldInfo" << std::endl;

  sphere_ = robot_->getFromDef("sphere");
  cube_ = robot_->getFromDef("cube");
  ned_robot_arm_ = robot_->getFromDef("Ned");

  ned_robot_arm_x_ = ned_robot_arm_->getField("translation")->getSFVec3f()[0];
  ned_robot_arm_y_ = ned_robot_arm_->getField("translation")->getSFVec3f()[1];

  joint_1_ = robot_->getMotor("joint_1");
  joint_2_ = robot_->getMotor("joint_2");
  joint_3_ = robot_->getMotor("joint_3");
  joint_5_ = robot_->getMotor("joint_5");
  joint_6_ = robot_->getMotor("joint_6");
  joint_base_to_jaw_1_ = robot_->getMotor("joint_base_to_jaw_1");
  joint_base_to_jaw_2_ = robot_->getMotor("joint_base_to_jaw_2");

  joint_1_sensor_ = robot_->getPositionSensor("joint_1_sensor");
  joint_2_sensor_ = robot_->getPositionSensor("joint_2_sensor");
  joint_3_sensor_ = robot_->getPositionSensor("joint_3_sensor");
  joint_5_sensor_ = robot_->getPositionSensor("joint_5_sensor");
  joint_6_sensor_ = robot_->getPositionSensor("joint_6_sensor");
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
  fillIdStringMaps({ "h", "c", "s", "position", "holding", "grab", "release", "move" });
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["holding"], tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID, { 1 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["c"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["s"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  

  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["grab"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["release"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["move"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));

  sendSetupMessage(objects, commands);
  waitForStartMsg();

  init();
  // Step once to initialize joint sensors.
  robot_->step(robot_time_step_);

  double h_position = getAngularPosition(joint_1_sensor_->getValue());
  double c_position = getPosition(cube_->getField("translation")->getSFVec3f());
  double s_position = getPosition(sphere_->getField("translation")->getSFVec3f());

  std::cout << "Sending initial data message with following data:" << std::endl;
  std::cout << "Hand Position: " << h_position << std::endl;
  std::cout << "Cube Position: " << c_position << std::endl;
  std::cout << "Sphere Position: " << s_position << std::endl;

  std::vector<tcp_io_device::MsgData> data_to_send;
  data_to_send.push_back(createMsgData<double>(objects[0], { h_position }));
  data_to_send.push_back(createMsgData<communication_id_t>(objects[1], { -1 }));
  data_to_send.push_back(createMsgData<double>(objects[2], { c_position }));
  data_to_send.push_back(createMsgData<double>(objects[3], { s_position }));
  sendDataMessage(data_to_send);
  state_ = IDLE;
  run();
}

void HandGrabSphereController::init() {

  // Decrease the PID gain from 10 so that we keep a grip on the object.
  joint_1_->setControlPID(9, 0, 0);
  // Increase the strength of the grip.
  joint_base_to_jaw_1_->setControlPID(250, 0, 0);
  joint_base_to_jaw_2_->setControlPID(250, 0, 0);

  joint_1_sensor_->enable(robot_time_step_);
  joint_2_sensor_->enable(robot_time_step_);
  joint_base_to_jaw_1_sensor_->enable(robot_time_step_);
  joint_base_to_jaw_2_sensor_->enable(robot_time_step_);

  joint_1_->setPosition((15 + position_offset_) * position_factor_);
  joint_2_->setPosition(arm_up_);
  joint_3_->setPosition(0.32);
  joint_5_->setPosition(-0.5);
  joint_6_->setPosition(3.1415296 / 2);

  joint_base_to_jaw_1_->setPosition(jaw_open_);
  joint_base_to_jaw_2_->setPosition(jaw_open_);

  sphere_->resetPhysics();
  cube_->resetPhysics();
  sphere_->getField("rotation")->setSFRotation(sphere_rotation_);
  sphere_->getField("translation")->setSFVec3f(sphere_translation_);
  cube_->getField("rotation")->setSFRotation(cube_rotation_);
  cube_->getField("translation")->setSFVec3f(cube_translation_);
}

void HandGrabSphereController::run() {
  int aera_us = 0;
#ifdef DEBUG
  diagnostic_mode_ = true;
#endif // DEBUG
  int receive_deadline = MAXINT;
  while (robot_->step(robot_time_step_) != -1) {
    if (!aera_started_) {
      std::cout << "AERA not started, wait for start message before calling run()" << std::endl;
      break;
    }
    auto msg = receive_queue_->dequeue();
    if (!msg && diagnostic_mode_ && aera_us >= receive_deadline)
    {
      while (!msg)
      {
        msg = receive_queue_->dequeue();
      }
    }
    if (msg) {
      receive_deadline = MAXINT;
      if (msg->messagetype() == tcp_io_device::TCPMessage_Type_DATA) {
        handleDataMsg(dataMsgToMsgData(std::move(msg)));
      }
      else {
        std::cout << "Received message with unexpected type "
          << tcp_io_device::TCPConnection::type_to_name_map_[msg->messagetype()]
          << ". Ignoring the message..." << std::endl;
      }
    }


    double h_position = getAngularPosition(joint_1_sensor_->getValue());

    if (aera_us == 1700 * 1000 + 40000) {
      //reset
      init();
    }
    // Don't send the state at time 0, but wait for the initial position.
    if (aera_us > 0 && aera_us % 100'000 == 0) {
      const double* c_translation = cube_->getField("translation")->getSFVec3f();
      const double* s_translation = sphere_->getField("translation")->getSFVec3f();
      double c_position = getPosition(c_translation);
      double s_position = getPosition(s_translation);

      communication_id_t holding_id = -1;
      if (fabs(joint_base_to_jaw_1_sensor_->getValue() - jaw_closed_) < 0.0005 &&
        fabs(joint_base_to_jaw_2_sensor_->getValue() - jaw_closed_)) {
        // The gripper is in the closed position. Check if an object is at the hand position with elevated Z.
        std::cout << "Hand Position: " << h_position << std::endl;
        std::cout << "Cube Position: " << c_position << std::endl;
        std::cout << "Sphere Position: " << s_position << std::endl;
        if (c_position == h_position && c_translation[2] > 0.014)
          holding_id = string_id_mapping_["c"];
        if (s_position == h_position && s_translation[2] > 0.014)
          holding_id = string_id_mapping_["s"];
      }

      std::cout << "Holding: " << holding_id << std::endl;

      std::vector<tcp_io_device::MsgData> msg_data;
      for (auto it = objects_meta_data_.begin(); it != objects_meta_data_.end(); ++it) {
        if (id_string_mapping_[it->getEntityID()] == "c" && id_string_mapping_[it->getID()] == "position")
        {
          msg_data.push_back(createMsgData<double>(*it, { c_position }));
          continue;
        }
        if (id_string_mapping_[it->getEntityID()] == "s" && id_string_mapping_[it->getID()] == "position")
        {
          msg_data.push_back(createMsgData<double>(*it, { s_position }));
          continue;
        }
        if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "position")
        {
          msg_data.push_back(createMsgData<double>(*it, { h_position }));
          continue;
        }
        if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "holding")
        {
          msg_data.push_back(createMsgData<communication_id_t>(*it, { holding_id }));
          continue;
        }
      }
      sendDataMessage(msg_data);

      receive_deadline = aera_us + 65000;
    }
    executeCommand();
    aera_us += robot_time_step_ * 100;
  }
}

void HandGrabSphereController::executeCommand() {
  double joint_1_pos = joint_1_sensor_->getValue();
  switch (state_)
  {
  case STARTING:
  case IDLE:
  case STOPPING:
    return;
  case MOVE_DOWN_CLOSE:
    joint_2_->setPosition(arm_down_);
    if (abs(arm_down_ - joint_2_sensor_->getValue()) <= position_accuracy_error_) {
      state_ = CLOSE_GRIPPER;
    }
    return;
  case MOVE_DOWN_OPEN:
    joint_2_->setPosition(arm_down_);
    if (abs(arm_down_ - joint_2_sensor_->getValue()) <= position_accuracy_error_) {
      state_ = OPEN_GRIPPER;
    }
    return;
  case MOVE_UP:
    joint_2_->setPosition(arm_up_);
    if (abs(arm_up_ - joint_2_sensor_->getValue()) <= position_accuracy_error_) {
      state_ = IDLE;
    }
    return;
  case MOVE_ARM:
    joint_1_->setPosition(joint_1_pos + (target_h_position_ * position_factor_));
    state_ = IDLE;
    return;
  case CLOSE_GRIPPER:
    joint_base_to_jaw_1_->setPosition(jaw_closed_);
    joint_base_to_jaw_2_->setPosition(jaw_closed_);
    if (abs(jaw_closed_ - joint_base_to_jaw_1_sensor_->getValue()) <= position_accuracy_error_ &&
      abs(jaw_closed_ - joint_base_to_jaw_2_sensor_->getValue()) <= position_accuracy_error_) {
      state_ = MOVE_UP;
    }
    return;
  case OPEN_GRIPPER:
    joint_base_to_jaw_1_->setPosition(jaw_open_);
    joint_base_to_jaw_2_->setPosition(jaw_open_);
    if (abs(jaw_open_ - joint_base_to_jaw_1_sensor_->getValue()) <= position_accuracy_error_ &&
      abs(jaw_open_ - joint_base_to_jaw_2_sensor_->getValue()) <= position_accuracy_error_) {
      state_ = MOVE_UP;
    }
    return;
  default:
    return;
  }

}

double HandGrabSphereController::getPosition(const double* translation) {

  double offset_x = translation[0] - ned_robot_arm_x_;
  double offset_y = translation[1] - ned_robot_arm_y_;
  double angle = atan2(offset_x, -offset_y);

  return getAngularPosition(angle);
}

double HandGrabSphereController::getAngularPosition(double angle) {
  double position = angle / position_factor_ - position_offset_;
  // Quantize.
  return floor((position + bin_size_ / 2) / bin_size_) * bin_size_;
}

void HandGrabSphereController::handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) {
  std::cout << "Received message with the following objects:" << std::endl;
  for (auto it = msg_data.begin(); it != msg_data.end(); ++it) {
    std::string id_name = id_string_mapping_[it->getMetaData().getID()];
    std::string entity_name = id_string_mapping_[it->getMetaData().getEntityID()];
    std::cout << "Entity: " + entity_name + " Property: " + id_name << std::endl;

    if (entity_name != "h") {
      // Unknown command
      continue;
    }
    if (id_name == "grab")
    {
      state_ = MOVE_DOWN_CLOSE;
    }
    else if (id_name == "release")
    {
      state_ = MOVE_DOWN_OPEN;
    }
    else if (id_name == "move")
    {
      target_h_position_ = it->getData<double>()[0];
      std::cout << "Moving arm to " << target_h_position_ << std::endl;
      state_ = MOVE_ARM;
    }
  }
}