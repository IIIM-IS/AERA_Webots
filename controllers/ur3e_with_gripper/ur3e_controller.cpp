#include "ur3e_controller.h"

#define DEBUG 1

UR3eController::UR3eController() : AERAController() {

  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  ur3e_robot_arm_ = robot_->getFromDef("UR10e");
  ur_kinematics_ = new universalRobots::UR(universalRobots::UR10, true, -0.17);

  arm_motors_[0] = robot_->getMotor("shoulder_pan_joint");
  arm_motors_[1] = robot_->getMotor("shoulder_lift_joint");
  arm_motors_[2] = robot_->getMotor("elbow_joint");
  arm_motors_[3] = robot_->getMotor("wrist_1_joint");
  arm_motors_[4] = robot_->getMotor("wrist_2_joint");
  arm_motors_[5] = robot_->getMotor("wrist_3_joint");

  arm_sensors_[0] = robot_->getPositionSensor("shoulder_pan_joint_sensor");
  arm_sensors_[1] = robot_->getPositionSensor("shoulder_lift_joint_sensor");
  arm_sensors_[2] = robot_->getPositionSensor("elbow_joint_sensor");
  arm_sensors_[3] = robot_->getPositionSensor("wrist_1_joint_sensor");
  arm_sensors_[4] = robot_->getPositionSensor("wrist_2_joint_sensor");
  arm_sensors_[5] = robot_->getPositionSensor("wrist_3_joint_sensor");

  hand_motors_[0] = robot_->getMotor("finger_1_joint_1");
  hand_motors_[1] = robot_->getMotor("finger_2_joint_1");
  hand_motors_[2] = robot_->getMotor("finger_middle_joint_1");

  hand_sensors_[0] = robot_->getPositionSensor("finger_1_joint_1_sensor");
  hand_sensors_[1] = robot_->getPositionSensor("finger_2_joint_1_sensor");
  hand_sensors_[2] = robot_->getPositionSensor("finger_middle_joint_1_sensor");

  boxes_[0] = robot_->getFromDef("red_box");
  boxes_[1] = robot_->getFromDef("green_box");
  boxes_[2] = robot_->getFromDef("blue_box");


  gps_sensor_ = robot_->getGPS("gps_tip");

  tip_camera_ = robot_->getCamera("tip_camera");

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_motors_[i]->setControlPID(2, 0, 0);
  }

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_sensors_[i]->enable(robot_time_step_);
  }
  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_sensors_[i]->enable(robot_time_step_);
  }

  gps_sensor_->enable(robot_time_step_);

  tip_camera_->enable(robot_time_step_);

  hand_closed_values_ = { 0.5, 0.5, 0.5 };
  for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_open_values_.push_back(hand_motors_[i]->getMinPosition());
  }
}

UR3eController::~UR3eController() {
  if (robot_) {
    delete robot_;
  }
}

int UR3eController::start() {
  
  std::vector<tcp_io_device::MetaData> objects;
  std::vector<tcp_io_device::MetaData> commands;

  std::vector<std::string> entities_objects;
  // Add entities to the vector.
  entities_objects.push_back("h");
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    entities_objects.push_back("b_" + std::to_string(i));
  }
  // Add properties to the vector
  entities_objects.push_back("position");
  entities_objects.push_back("holding");
  // Add commands to the vector
  entities_objects.push_back("grab");
  entities_objects.push_back("release");
  entities_objects.push_back("move");
  // Generate communication ids by filling the string_id_mapping_
  fillIdStringMaps(entities_objects);

  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 2 }));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["holding"], tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID, { 1 }));
  
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    objects.push_back(tcp_io_device::MetaData(string_id_mapping_["b_" + std::to_string(i)], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 2 }));
  }

  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["grab"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["release"], tcp_io_device::VariableDescription_DataType_BYTES, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["move"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 1 }));
  

  //sendSetupMessage(objects, commands);
  //waitForStartMsg();
  init();

  const double* hand_pos_array = gps_sensor_->getValues();
  std::vector<double> hand_xy_pos = getRoundedXYPosition(hand_pos_array);

  std::vector<std::vector<double>> box_xy_positions;
  std::vector<double> box_z_positions;
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
    box_z_positions.push_back(round(box_pos_array[2] * 10.) / 10.);
    std::vector<double> box_xy_pos = getRoundedXYPosition(box_pos_array);
    box_xy_positions.push_back(box_xy_pos);
  }

  std::vector<tcp_io_device::MsgData> data_to_send;
  data_to_send.push_back(createMsgData<double>(objects[0], hand_xy_pos));
  data_to_send.push_back(createMsgData<communication_id_t>(objects[1], { -1 }));
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    data_to_send.push_back(createMsgData<double>(objects[2 + i], box_xy_positions[i]));
  }
  // sendDataMessage(data_to_send);
  state_ = IDLE;
  run();
  return 0;
}

void UR3eController::init() {
  robot_->step(robot_time_step_);
  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i)
  {
    arm_motors_[i]->setPosition(0.0 + motor_offsets_[i]);
  }

  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i)
  {
    hand_motors_[i]->setPosition(hand_motors_[i]->getMinPosition());
  }
  double random_positions[NUMBER_OF_BOXES][3];

  get_random_box_positions(random_positions);

  for (size_t i = 0; i < NUMBER_OF_BOXES; i++)
  {
    boxes_[i]->getField("rotation")->setSFRotation(box_rotations_[i]);
    boxes_[i]->getField("translation")->setSFVec3f(random_positions[i]);
  }
}

void UR3eController::run() {

  int aera_us = 100'000;
#ifdef DEBUG
  diagnostic_mode_ = true;
#endif // DEBUG
  int receive_deadline = MAXINT;

  bool first = true;
  bool second = true;
  while (robot_->step(robot_time_step_) != -1) {
    /*
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    */
    const double* hand_pos_array = gps_sensor_->getValues();
    std::vector<double> hand_xy_pos = getRoundedXYPosition(hand_pos_array);

    std::vector<std::vector<double>> box_xy_positions;
    std::vector<double> box_z_positions;
    for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
      const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
      box_z_positions.push_back(round(box_pos_array[2] * 10.) / 10.);
      std::vector<double> box_xy_pos = getRoundedXYPosition(box_pos_array);
      box_xy_positions.push_back(box_xy_pos);
    }

    communication_id_t holding_id = -1;
    for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
      if (sqrt((hand_xy_pos[0] - box_xy_positions[i][0]) * (hand_xy_pos[0] - box_xy_positions[i][0]) +
        (hand_xy_pos[1] - box_xy_positions[i][1]) * (hand_xy_pos[1] - box_xy_positions[i][1])) > 0.1) {
        continue;
      }
      if (box_z_positions[i] > 0.014)
      {
        holding_id = string_id_mapping_["b_" + std::to_string(i)];
        break;
      }
    }

    std::vector<tcp_io_device::MsgData> msg_data;
    for (auto it = objects_meta_data_.begin(); it != objects_meta_data_.end(); ++it) {
      if (id_string_mapping_[it->getID()] == "holding") {
        msg_data.push_back(createMsgData<communication_id_t>(*it, { holding_id }));
        continue;
      }
      if (id_string_mapping_[it->getID()] == "position") {
        if (id_string_mapping_[it->getEntityID()] == "h") {
          msg_data.push_back(createMsgData<double>(*it, hand_xy_pos));
          continue;
        }
        std::string entity = id_string_mapping_[it->getEntityID()];
        if (entity.substr(0, 2) == "b_") {
          int box_num = std::stoi(entity.substr(2));
          msg_data.push_back(createMsgData<double>(*it, box_xy_positions[box_num]));
          continue;
        }
      }
      //sendDataMessage(msg_data);

      receive_deadline = aera_us + 65000;
    }

    if (first) {
      target_h_position_ = box_xy_positions[0];
      state_ = MOVE_ARM;
      first = false;
    }

    if (second && !first && state_ == IDLE) {
      target_h_position_ = box_xy_positions[1];
      state_ = MOVE_ARM;
      second = false;
    }

    executeCommand();
    aera_us += robot_time_step_ * 100;
  }
}


void UR3eController::executeCommand() {
  switch (state_)
  {
  case STARTING:
  case IDLE:
  case STOPPING:
    return;
  case MOVE_DOWN_CLOSE:
    target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 0);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_CLOSE;
        break;
      }
      state_ = CLOSE_GRIPPER;
    }
    return;
  case MOVE_DOWN_OPEN:
    target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 0);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_OPEN;
        break;
      }
      state_ = OPEN_GRIPPER;
    }
    return;
  case MOVE_UP:
    target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_UP;
        break;
      }
      state_ = IDLE;
    }
    return;
  case MOVE_ARM:
    target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_ARM;
        break;
      }
      state_ = MOVE_DOWN_CLOSE;
    }
    return;
  case CLOSE_GRIPPER:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_closed_values_[i] - hand_sensors_[i]->getValue()) > position_accuracy_error_) {
        setHandAngles(hand_closed_values_);
        state_ = CLOSE_GRIPPER;
        break;
      }
      state_ = MOVE_UP;
    }
    return;
  case OPEN_GRIPPER:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_open_values_[i] - hand_sensors_[i]->getValue()) > position_accuracy_error_) {
        setHandAngles(hand_open_values_);
        state_ = OPEN_GRIPPER;
        break;
      }
      state_ = MOVE_UP;
    }
    return;
  default:
    return;
  }

}


void UR3eController::handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) {
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
      target_h_position_ = it->getData<double>();
      std::cout << "Moving arm to: x: " << target_h_position_[0] << ", y: " << target_h_position_[1] << std::endl;
      state_ = MOVE_ARM;
    }
  }
}


std::vector<double> UR3eController::getJointAnglesFromXY(std::vector<double> xy_pos, int z_level) {

  float joint_angles_array[8][6];

  ur_kinematics_->inverseKinematics(universalRobots::pose(xy_pos[0], xy_pos[1], 0.2 * z_level, M_PI, 0.0, 0.0), &joint_angles_array);

  std::vector<double> out;
  for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    out.push_back(joint_angles_array[SOLUTION_TO_CHOOSE][i]);
  }
  for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    if (isnan(out[i])) {
      std::cout << "ERROR: COULD NOT GET VALID SOLUTION TO INVERSE KINEMATIC, NOT MOVING..." << std::endl;
      return std::vector<double>();
    }
    out[i] += motor_offsets_[i];
  }
  return out;
}

void UR3eController::setJointAngles(std::vector<double> joint_angles) {
  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_motors_[i]->setPosition(joint_angles[i]);
  }
}

void UR3eController::setHandAngles(std::vector<double> hand_values) {

  for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_motors_[i]->setPosition(hand_values[i]);
  }
}