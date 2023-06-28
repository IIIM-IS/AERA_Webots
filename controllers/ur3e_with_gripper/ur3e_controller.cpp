#include "ur3e_controller.h"

#define DEBUG 1

UR3eController::UR3eController() : AERAController() {

  state_ = STARTING;
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
    arm_motors_[i]->setVelocity(1.0);
  }

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_sensors_[i]->enable(robot_time_step_);
  }
  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_sensors_[i]->enable(robot_time_step_);
  }

  gps_sensor_->enable(robot_time_step_);

  tip_camera_->enable(robot_time_step_);

  hand_closed_values_ = { 0.4, 0.4, 0.4 };
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

  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 3 }, "vec3"));
  objects.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["holding"], tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID, { 1 }));
  
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    objects.push_back(tcp_io_device::MetaData(string_id_mapping_["b_" + std::to_string(i)], string_id_mapping_["position"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 3 }, "vec3"));
  }

  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["grab"], tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["release"], tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID, { 1 }));
  commands.push_back(tcp_io_device::MetaData(string_id_mapping_["h"], string_id_mapping_["move"], tcp_io_device::VariableDescription_DataType_DOUBLE, { 3 }, "vec3"));
  

  sendSetupMessage(objects, commands);
  waitForStartMsg();
  init();
  // Step once to initialize joint sensors.
  robot_->step(robot_time_step_);

  const double* hand_pos_array = gps_sensor_->getValues();
  hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);

  std::vector<std::vector<double>> box_xyz_positions;
  std::vector<double> box_z_positions;
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
    box_z_positions.push_back(round(box_pos_array[2] * 10.) / 10.);
    std::vector<double> box_xyz_pos = getRoundedXYZPosition(box_pos_array);
    box_xyz_positions.push_back(box_xyz_pos);
  }

  std::vector<tcp_io_device::MsgData> data_to_send;
  data_to_send.push_back(createMsgData<double>(objects[0], hand_xyz_pos_));
  data_to_send.push_back(createMsgData<communication_id_t>(objects[1], { -1 }));
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    data_to_send.push_back(createMsgData<double>(objects[2 + i], box_xyz_positions[i]));
  }
  sendDataMessage(data_to_send);
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

  for (size_t i = 1; i < NUMBER_OF_BOXES; i++)
  {
    boxes_[i]->getField("rotation")->setSFRotation(box_rotations_[i]);
    boxes_[i]->getField("translation")->setSFVec3f(random_positions[i]);
  }

  boxes_[0]->getField("rotation")->setSFRotation(box_rotations_[0]);
  boxes_[0]->getField("translation")->setSFVec3f(box_positions_[0]);
}

void UR3eController::run() {

  int aera_us = 100'000 / robot_time_step_;
#ifdef DEBUG
  diagnostic_mode_ = true;
#endif // DEBUG
  int receive_deadline = aera_us + 65'000 / robot_time_step_;
  std::unique_ptr<tcp_io_device::TCPMessage> pending_msg;
  while (robot_->step(robot_time_step_) != -1) {
    if (!aera_started_) {
      std::cout << "AERA not started, wait for start message before calling run()" << std::endl;
      break;
    }
    auto msg = receive_queue_->dequeue();
    if (!msg && diagnostic_mode_ && !pending_msg && aera_us == receive_deadline)
    {
      while (!msg)
      {
        msg = receive_queue_->dequeue();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    if (msg) {
      pending_msg = std::move(msg);
    }

    if (pending_msg && (!diagnostic_mode_ || aera_us >= receive_deadline)) {
      if (pending_msg->messagetype() == tcp_io_device::TCPMessage_Type_DATA) {
        handleDataMsg(dataMsgToMsgData(std::move(pending_msg)));
      }
      else {
        std::cout << "Received message with unexpected type "
          << tcp_io_device::TCPConnection::type_to_name_map_[msg->messagetype()]
          << ". Ignoring the message..." << std::endl;
      }
      pending_msg = NULL;
    }
    // Don't send the state on the first pass, but wait to arrive at the initial position.
    if (aera_us > 100'000 / robot_time_step_ && aera_us % (int)(100'000 / robot_time_step_) == 0) {
      const double* hand_pos_array = gps_sensor_->getValues();
      hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);

      std::vector<std::vector<double>> box_xyz_positions;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
        std::vector<double> box_xyz_pos = getRoundedXYZPosition(box_pos_array);
        box_xyz_positions.push_back(box_xyz_pos);
      }

      communication_id_t holding_id = -1;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        if (sqrt((hand_xyz_pos_[0] - box_xyz_positions[i][0]) * (hand_xyz_pos_[0] - box_xyz_positions[i][0]) +
          (hand_xyz_pos_[1] - box_xyz_positions[i][1]) * (hand_xyz_pos_[1] - box_xyz_positions[i][1])) > 0.1) {
          continue;
        }
        if (box_xyz_positions[i][2] > 0.014)
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
            msg_data.push_back(createMsgData<double>(*it, hand_xyz_pos_));
            continue;
          }
          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity.substr(0, 2) == "b_") {
            int box_num = std::stoi(entity.substr(2));
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[box_num]));
            continue;
          }
        }
      }
      std::cout << "Sending message with following data:" << std::endl;
      sendDataMessage(msg_data);
      for (int i = 0; i < msg_data.size(); ++i) {
        std::cout << msg_data[i] << std::endl;
      }
      receive_deadline = aera_us + 65'000 / robot_time_step_;
    }

    executeCommand();
    int next_aera_us = aera_us + robot_time_step_ * 100 / robot_time_step_;
    if (diagnostic_mode_ && state_ != IDLE && (next_aera_us % (int)(100'000 / robot_time_step_) == 0)) {
      // In the next interation, we would send the state, but the robot is still executing a command.
      // In diagnistic mode, don't advance aera_us but wait for IDLE until we send the state.
    }
    else
      aera_us = next_aera_us;
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
      state_ = IDLE;
    }
    return;
  case CLOSE_GRIPPER:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_closed_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        std::cout << "Distance to desired value: " << fabs(hand_closed_values_[i] - hand_sensors_[i]->getValue()) << std::endl;
        setHandAngles(hand_closed_values_);
        state_ = CLOSE_GRIPPER;
        break;
      }
      state_ = MOVE_UP;
    }
    return;
  case OPEN_GRIPPER:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_open_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
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
      std::cout << "Move command1" << std::endl;
      std::vector<double> move_by_command = it->getData<double>();
      std::cout << "Move command2" << std::endl;

      std::vector<double> target_position;
      std::cout << "Move command3" << std::endl;
      std::cout << "HandXYZPos: " << hand_xyz_pos_.size() << std::endl;
      for (int i = 0; i < move_by_command.size(); ++i) {
        std::cout << "Move command4" << std::endl;
        target_position.push_back(hand_xyz_pos_[i] + move_by_command[i]);
        std::cout << "Move command5" << std::endl;
      }
      
      target_h_position_ = target_position;

      std::cout << "Moving arm to: x: " << target_h_position_[0] << ", y: " << target_h_position_[1] << std::endl;
      
      state_ = MOVE_ARM;
    }
  }
}


std::vector<double> UR3eController::getJointAnglesFromXY(std::vector<double> xy_pos, int z_level) {

  float joint_angles_array[8][6];

  ur_kinematics_->inverseKinematics(universalRobots::pose(xy_pos[0], xy_pos[1], (0.185 * z_level) + 0.015, M_PI, 0.0, 0.0), &joint_angles_array);

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