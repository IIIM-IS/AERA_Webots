#include "hand_grab_sphere_UR_3D_controller.h"
#include "../AERA_controller_base/toml_parser.h"

#define DEBUG 1
#define Z_0 1

UR3eController::UR3eController() : AERAController() {

  state_ = STARTING;
  measurement_state_ = MEASURE_HAND;
  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  running_time_steps_ = 0;
  max_exec_time_steps_ = execution_times_map_.at("default");
  receive_cmd_time_ = INT_MAX - 1;
  ur3e_robot_arm_ = robot_->getFromDef("UR10e");
  ur_kinematics_ = new universalRobots::UR(universalRobots::UR10, true, -0.17);
  holding_id_ = -1;

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
  inertial_unit_ = robot_->getInertialUnit("inertial unit");
  tip_camera_ = new TipCamera(robot_->getCamera("tip_camera"));

  keyboard_ = robot_->getKeyboard();
  mouse_ = robot_->getMouse();

  gps_sensor_->enable(robot_time_step_);
  inertial_unit_->enable(robot_time_step_);
  tip_camera_->enable(robot_time_step_);

  keyboard_->enable(robot_time_step_);
  mouse_->enable(robot_time_step_);
  mouse_->enable3dPosition();
  mouse_pressed_ = false;

  hand_closed_values_ = { 0.4, 0.4, 0.4 };
  for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_open_values_.push_back(hand_motors_[i]->getMinPosition());
  }
}

UR3eController::~UR3eController() {
  if (robot_) {
    delete robot_;
  }
  if (tip_camera_) {
    delete tip_camera_;
  }
}

int UR3eController::start() {

  std::map<std::string, std::map<std::string, tcp_io_device::MetaData> > objects_map;

  objects_map = setup("settings.toml");

  waitForStartMsg();
  init();
  // Step once to initialize joint sensors.
  robot_->step(robot_time_step_);
  running_time_steps_ += robot_time_step_;
  robot_->step(robot_time_step_);
  running_time_steps_ += robot_time_step_;

  const double* hand_pos_array = gps_sensor_->getValues();
  hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);
#if 1
  hand_xyz_pos_[0] = 1;
  hand_xyz_pos_[1] = 0;
  hand_xyz_pos_[2] = 0.2;
#endif

  std::vector<std::vector<double>> box_xyz_positions;
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
    std::vector<double> box_xyz_pos = getRoundedXYZPosition(box_pos_array);
    box_xyz_positions.push_back(box_xyz_pos);
  }
  std::vector<tcp_io_device::MsgData> data_to_send;

  for (auto o = objects_map.begin(); o != objects_map.end(); ++o) {
    std::string entity = o->first;
    for (auto p = o->second.begin(); p != o->second.end(); ++p) {
      std::string prop = p->first;
      if (entity == "h") {
        if (prop == "position") {
#if 0
          data_to_send.push_back(createMsgData<double>(p->second, { hand_xyz_pos_[0], hand_xyz_pos_[1], 0. }));
#else
          data_to_send.push_back(createMsgData<double>(p->second, hand_xyz_pos_));
          last_sent_hand_xyz_pos_ = hand_xyz_pos_;
#endif
        }
        else if (prop == "rotation") {
          data_to_send.push_back(createMsgData<double>(p->second, std::vector<double>({ 0.5, -0.5, 0.5, -0.5 })));
        }
        else if (prop == "holding") {
          data_to_send.push_back(createMsgData<communication_id_t>(p->second, { -1 }));
        }
      }
      else if (entity == "h_sensor") {
        if (prop == "measurement") {
#if Z_0
          data_to_send.push_back(createMsgData<double>(p->second, { hand_xyz_pos_[0] - 0.1, hand_xyz_pos_[1] - 0.1, 0. }));
#else
          data_to_send.push_back(createMsgData<double>(p->second, hand_xyz_pos_));
#endif
        }
      }
      else if (entity.substr(0, 2) == "b_") {
        if (prop == "position") {
          std::vector<double> box_pos = box_xyz_positions[std::stoi(entity.substr(2, 3))];
#if Z_0
          data_to_send.push_back(createMsgData<double>(p->second, { box_pos[0], box_pos[1], 0. }));
#else
          data_to_send.push_back(createMsgData<double>(p->second, box_pos));
#endif
        }
      }
    }
  }

  sendDataMessage(data_to_send);
  state_ = IDLE;
  run();
  return 0;
}

void UR3eController::init() {
  robot_->step(robot_time_step_);
  running_time_steps_ += robot_time_step_;

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_motors_[i]->setVelocity(1.0);
  }

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
    arm_sensors_[i]->enable(robot_time_step_);
  }
  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    hand_sensors_[i]->enable(robot_time_step_);
  }

#if 0
  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i)
  {
    arm_motors_[i]->setPosition(0.0 + motor_offsets_[i]);
    if (target_joint_angles_.size() <= i) {
      target_joint_angles_.push_back(0.0 + motor_offsets_[i]);
    }
    else {
      target_joint_angles_[i] = 0.0 + motor_offsets_[i];
    }
  }

#elif 1
  target_h_position_ = std::vector<double>{ 1, 0, 0.2 };
  target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 1);

  for (size_t i = 0; i < NUMBER_OF_ARM_MOTORS; ++i)
  {
    arm_motors_[i]->setPosition(target_joint_angles_[i]);
  }
#endif

  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i)
  {
    hand_motors_[i]->setPosition(hand_motors_[i]->getMinPosition());
  }
  double random_positions[NUMBER_OF_BOXES][3];

  get_random_box_positions(random_positions);
#if 1 // debug
  random_positions[0][0] = -0.5;
  random_positions[0][1] = 0.7;
  random_positions[1][0] = 0;
  random_positions[1][1] = 1;
  random_positions[2][0] = 0;
  random_positions[2][1] = -1;
#endif

  for (size_t i = 0; i < NUMBER_OF_BOXES; i++)
  {
    boxes_[i]->getField("rotation")->setSFRotation(box_rotations_[i]);
    boxes_[i]->getField("translation")->setSFVec3f(random_positions[i]);
  }

#if 0
  boxes_[0]->getField("rotation")->setSFRotation(box_rotations_[0]);
  boxes_[0]->getField("translation")->setSFVec3f(box_positions_[0]);
#endif
}

void UR3eController::run() {

  int aera_us = 100'000 / robot_time_step_;
#ifdef DEBUG
  diagnostic_mode_ = true;
#endif // DEBUG
  int receive_deadline = aera_us + 65'000 / robot_time_step_;
  last_msg_sent_time_ = 0.0;
  std::unique_ptr<tcp_io_device::TCPMessage> pending_msg;
  while (robot_->step(robot_time_step_) != -1) {
    running_time_steps_ += robot_time_step_;
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
        auto m = acceptNewUserInput();
        robot_->step(robot_time_step_ * 10);
        if (!m.isValid())
          continue;
        sendGoalMessage(m);
        std::cout << "Following goal message was accepted: " << std::endl << m << std::endl;
        break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } 
    }
    if (msg) {
      std::cout << "msg received in controller of type: " << msg->messagetype() << std::endl;
      pending_msg = std::move(msg);
    }
    if (pending_msg) { // && (!diagnostic_mode_ || aera_us == receive_deadline)) {
      switch (pending_msg->messagetype())
      {
      case tcp_io_device::TCPMessage_Type_DATA:
        handleDataMsg(dataMsgToMsgData(std::move(pending_msg)));
        std::cout << "HandleDataMsg called" << std::endl;
        robot_->step(1);
        break;
      case tcp_io_device::TCPMessage_Type_RECONNECT:
        std::cout << "Reconnect occured, resetting environment and sending new setup message" << std::endl;
        handleReconnect();
      default:
        std::cout << "Received message with unexpected type "
          << tcp_io_device::TCPConnection::type_to_name_map_[pending_msg->messagetype()]
          << ". Ignoring the message..." << std::endl;
        break;
      }
      pending_msg = NULL;
    }
    // Don't send the state on the first pass, but wait to arrive at the initial position.
    if (aera_us > 100'000 / robot_time_step_ && (aera_us % (int)(100'000 / robot_time_step_)) == 0 || true) {

      const double* hand_pos_array = gps_sensor_->getValues();
      hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);

      const double* hand_rotation = inertial_unit_->getQuaternion();
      double _1 = hand_rotation[3] < 0 ? -1 : 1;
      Eigen::Quaterniond new_rotation = Eigen::Quaterniond(
        _1 * round(hand_rotation[3] * 100.) / 100.,
        _1 * round(hand_rotation[0] * 100.) / 100.,
        _1 * round(hand_rotation[1] * 100.) / 100.,
        _1 * round(hand_rotation[2] * 100.) / 100.);
      new_rotation.normalize();

      //std::cout << "new_rotation: " << new_rotation << std::endl;
      //std::cout << "old_rotation: " << hand_rotation_ << std::endl;

      if (!(new_rotation.coeffs() == (hand_rotation_.coeffs() * -1.) || new_rotation == hand_rotation_)) {
        hand_rotation_ = new_rotation;
      }

      // std::cout << "old_rotation: " << hand_rotation_ << std::endl;

      std::vector<std::vector<double>> box_xyz_positions;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        const double* box_pos_array = boxes_[i]->getField("translation")->getSFVec3f();
        std::vector<double> box_xyz_pos = getRoundedXYZPosition(box_pos_array);
        box_xyz_pos = add_gaussian_noise(box_xyz_pos, 0.02);
        box_xyz_positions.push_back(box_xyz_pos);
      }

      holding_id_ = -1;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        if (sqrt((hand_xyz_pos_[0] - box_xyz_positions[i][0]) * (hand_xyz_pos_[0] - box_xyz_positions[i][0]) +
          (hand_xyz_pos_[1] - box_xyz_positions[i][1]) * (hand_xyz_pos_[1] - box_xyz_positions[i][1])) > 0.1) {
          continue;
        }
        if (box_xyz_positions[i][2] > 0.14)
        {
          holding_id_ = string_id_mapping_["b_" + std::to_string(i)];
          break;
        }
      }

      tip_camera_->recognitionEnable(1);
      robot_->step(1);

      std::vector<tcp_io_device::MsgData> msg_data;
      for (auto it = objects_meta_data_.begin(); it != objects_meta_data_.end(); ++it) {
        if (id_string_mapping_[it->getID()] == "holding") {
          msg_data.push_back(createMsgData<communication_id_t>(*it, { holding_id_ }));
          continue;
        }
        if (id_string_mapping_[it->getID()] == "color") {
          std::vector<double> color = tip_camera_->getColorOfObject();
          if (color.size() == 0) {
            continue;
          }
          std::vector<int64_t> color_to_send;
          //std::cout << "Color to send: ";
          for (int i = 0; i < color.size(); ++i) {
            color_to_send.push_back((int)(color[i]));
            std::cout << color_to_send[i];
          }
          std::cout << std::endl;
          //msg_data.push_back(createMsgData(*it, std::to_string(color_to_send[0])));
          continue;
        }
        if (id_string_mapping_[it->getID()] == "rotation") {

          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity == "h") {

            std::vector<double> data_to_send;
            data_to_send.push_back(round(hand_rotation_.w() * 100.) / 100.);
            data_to_send.push_back(round(hand_rotation_.x() * 100.) / 100.);
            data_to_send.push_back(round(hand_rotation_.y() * 100.) / 100.);
            data_to_send.push_back(round(hand_rotation_.z() * 100.) / 100.);
            //msg_data.push_back(createMsgData<double>(*it, data_to_send));
          }
          else if (entity.substr(0, 2) == "b_") {
            if (measurement_state_ != MEASURE_HAND) {
              //msg_data.push_back(createMsgData<double>(*it, std::vector<double>()));
              continue;
            }
            int number_of_rec_objects = tip_camera_->getRecognitionNumberOfObjects();

            if (number_of_rec_objects != 0) {
              auto rec_objects = tip_camera_->getRecognitionObjects();

              rec_objects[0].id;
              boxes_[0]->getId();

              if (rec_objects[0].id != boxes_[std::stoi(entity.substr(2, 3))]->getId()) {
                std::cout << "ID of rec_obj: " << rec_objects[0].id << std::endl;
                std::cout << "ID of checked obj: " << boxes_[std::stoi(entity.substr(2, 3))]->getId() << std::endl;
                std::cout << "box0 id: " << boxes_[0]->getId() << ", box1 id: " << boxes_[1]->getId() << ", box2 id: " << boxes_[2]->getId() << std::endl;
                continue;
              }

              auto orientation = rec_objects[0].orientation;
              Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(orientation[3], Eigen::Vector3d(orientation[0], orientation[1], orientation[2]));
              Eigen::Quaterniond in_inertial_frame = angle_axis.inverse() * Eigen::AngleAxisd(- M_PI / 2., Eigen::Vector3d(0, 0, 1));

              std::cout << "Orientation of object in inertial frame:" << std::endl << "Quaternion: " << std::endl << in_inertial_frame << std::endl;

              std::vector<double> data_to_send;
              data_to_send.push_back(round(in_inertial_frame.w() * 100.) / 100.);
              data_to_send.push_back(round(in_inertial_frame.x() * 100.) / 100.);
              data_to_send.push_back(round(in_inertial_frame.y() * 100.) / 100.);
              data_to_send.push_back(round(in_inertial_frame.z() * 100.) / 100.);
              //msg_data.push_back(createMsgData<double>(*it, data_to_send));
            }
          }
        }
        if (id_string_mapping_[it->getID()] == "measurement") {
          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity == "h_sensor") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = hand_xyz_pos_[2];
            hand_xyz_pos_[2] = add_gaussian_noise({0.0}, 0.02)[0];

            std::vector<double> h_s_pos{ hand_xyz_pos_[0] - 0.1, hand_xyz_pos_[1] - 0.1, hand_xyz_pos_[2] };
#endif
            msg_data.push_back(createMsgData<double>(*it, h_s_pos));
#if Z_0
            hand_xyz_pos_[2] = save_z;
#endif
            //measurement_state_ = NONE;
            continue;
          }
        }
        if (id_string_mapping_[it->getID()] == "position") {
          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity == "h") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = hand_xyz_pos_[2];
            hand_xyz_pos_[2] = add_gaussian_noise({ 0.0 }, 0.02)[0];
#endif
            msg_data.push_back(createMsgData<double>(*it, hand_xyz_pos_));
#if Z_0
            hand_xyz_pos_[2] = save_z;
#endif
            //measurement_state_ = NONE;
            continue;
          }
          if (entity == "cam") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = hand_xyz_pos_[2];
            hand_xyz_pos_[2] = 0;
#endif
            //msg_data.push_back(createMsgData<double>(*it, hand_xyz_pos_));
#if Z_0
            hand_xyz_pos_[2] = save_z;
#endif
            continue;
          }
          if (entity.substr(0, 2) == "b_") {
            int box_num = std::stoi(entity.substr(2));
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[box_num][2];
            box_xyz_positions[box_num][2] = add_gaussian_noise({ 0.0 }, 0.02)[0];
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[box_num]));
#if Z_0
            box_xyz_positions[box_num][2] = save_z;
#endif
            continue;
          }
        }
      }
      tip_camera_->recognitionDisable();
      // std::cout << "Sending message with following data:" << std::endl;
      // if (robot_->getTime() - last_msg_sent_time_ > 0.005) {
        sendDataMessage(msg_data);
        last_sent_hand_xyz_pos_ = hand_xyz_pos_;
        // last_msg_sent_time_ = robot_->getTime();
      // }
      // for (int i = 0; i < msg_data.size(); ++i) {
      //   std::cout << msg_data[i] << std::endl;
      // }
      measurement_state_ = NONE;
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
  std::cout << "Executing command with state_: " << state_ << std::endl;
  switch (state_)
  {
  case STARTING:
  case IDLE:
  case STOPPING:
    return;
  case MOVE_DOWN_CLOSE:
    std::cout << 1 << std::endl;
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = MOVE_UP;
      return;
    }
    std::cout << 2 << std::endl;
    target_joint_angles_ = getJointAnglesFromXY(target_h_position_, 0);
    std::cout << 3 << std::endl;
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_CLOSE;
        break;
      }
      state_ = CLOSE_GRIPPER;
      std::cout << 4 << std::endl;
    }
    std::cout << 5 << std::endl;
    return;
  case MOVE_DOWN_OPEN:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = MOVE_UP;
      return;
    }
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
        std::cout << "Arm joint " << i << " above precision_accuracy error: " << fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) << std::endl;
        std::cout << "Target angle: " << target_joint_angles_[i] << std::endl;
        std::cout << "Arm sensor angle: " << arm_sensors_[i]->getValue() << std::endl;
        setJointAngles(target_joint_angles_);
        state_ = MOVE_UP;
        break;
      }
      if (holding_id_ == -1) {
        state_ = OPEN_GRIPPER_SAFE;
      }
      else {
        state_ = IDLE;
      }
    }
    return;
  case MOVE_ARM:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = IDLE;
      return;
    }
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
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = OPEN_GRIPPER;
      return;
    }
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_closed_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
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
  case OPEN_GRIPPER_SAFE:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_open_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_open_values_);
        state_ = OPEN_GRIPPER_SAFE;
        break;
      }
      state_ = IDLE;
    }
    return;
  case ROTATE_HAND:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = IDLE;
      return;
    }
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = ROTATE_HAND;
        break;
      }
      state_ = IDLE;
    }
    return;
  case MEASURING:
  default:
    return;
  }

}

tcp_io_device::MsgData UR3eController::acceptNewUserInput() {
  int pressed_key = keyboard_->getKey();
  if (pressed_key != 'G') {
    return tcp_io_device::MsgData::invalidMsgData();
  }
  auto mouse_state = mouse_->getState();

  // Only continues when mouse pressed is true and the current state false. Implements "onClickRelease" behavior.
  if (!(mouse_pressed_ && !mouse_state.left)) {
    mouse_pressed_ = mouse_state.left;
    return tcp_io_device::MsgData::invalidMsgData();
  }

  mouse_pressed_ = mouse_state.left;

  std::cout << "G+Click detected at:" << std::endl;
  std::cout << "u: " << mouse_state.u << std::endl;
  std::cout << "v: " << mouse_state.v << std::endl;
  std::cout << "x: " << mouse_state.x << std::endl;
  std::cout << "y: " << mouse_state.y << std::endl;
  std::cout << "z: " << mouse_state.z << std::endl;

  const webots::Node* selected_node = robot_->getSelected();

  if (!selected_node) {
    // Outside the arena.
    return tcp_io_device::MsgData::invalidMsgData();
  }

  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    if (boxes_[i] != selected_node) {
      continue;
    }
    std::cout << "Selected Box" << i << std::endl;


    std::vector<double> rounded_box_position = getRoundedXYZPosition(boxes_[i]->getField("translation")->getSFVec3f());
    std::vector<double> rounded_hand_position = getRoundedXYZPosition(&(hand_xyz_pos_[0]));
#if 1
    std::vector<double> delta = { rounded_box_position[0] - rounded_hand_position[0], rounded_box_position[1] - rounded_hand_position[1], 0.0 };
#else
    std::vector<double> delta = { rounded_box_position[0] - rounded_hand_position[0], rounded_box_position[1] - rounded_hand_position[1], rounded_box_position[2] - rounded_hand_position[2] };
#endif

    if (boxes_[i]->getField("translation")->getSFVec3f()[2] >= 0.14) {
      // Case: Clicked on box in hand of robot -> Release goal is sent to AERA.
      for (auto it = commands_meta_data_.begin(); it != commands_meta_data_.end(); ++it) {
        std::cout << "Checking for meta_data_object. Meta data object found: Entity: " << id_string_mapping_[it->getEntityID()] << ", property: " << id_string_mapping_[it->getID()] << std::endl;
        if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "release") {
          tcp_io_device::MsgData goal_proto_var = tcp_io_device::MsgData::createNewMsgData(*it, std::vector<communication_id_t>{it->getEntityID()});
          return goal_proto_var;
        }
      }
      std::cout << "WARNING: No MetaData for release command found. This is some setup issue." << std::endl;
      return tcp_io_device::MsgData::invalidMsgData();
    }
      
    if (fabs(delta[0]) <= 0.01 && fabs(delta[1]) <= 0.01) {
      // Case: Clicked box beneath hand. Send grab command as goal to AERA.
      for (auto it = commands_meta_data_.begin(); it != commands_meta_data_.end(); ++it) {
        if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "grab") {
          tcp_io_device::MsgData goal_proto_var = tcp_io_device::MsgData::createNewMsgData(*it, std::vector<communication_id_t>{it->getEntityID()});
          return goal_proto_var;
        }
      }
      std::cout << "WARNING: No MetaData for grab command found. This is some setup issue." << std::endl;
      return tcp_io_device::MsgData::invalidMsgData();

    }

    // Case: Otherwise move to the box

    for (auto it = commands_meta_data_.begin(); it != commands_meta_data_.end(); ++it) {
      if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "move") {
        tcp_io_device::MsgData goal_proto_var = tcp_io_device::MsgData::createNewMsgData(*it, std::vector<double>{ delta[0], delta[1], delta[2] });
        return goal_proto_var;
      }
    }
    std::cout << "WARNING: No MetaData for move command found. This is some setup issue." << std::endl;
    return tcp_io_device::MsgData::invalidMsgData();
  }

  // Case: click on the arena: Move to the mouse position.

  std::cout << "selected_node->getTypeName(): " << selected_node->getTypeName() << std::endl;
  if (selected_node->getTypeName() != "CircleArena") {
    // Other than clicking on boxes, only register clicks inside the arena.
    return tcp_io_device::MsgData::invalidMsgData();
  }

  if (isnan(mouse_state.x) || isnan(mouse_state.y) || isnan(mouse_state.z)) {
    std::cout << "WARNING: Registered nan in mouse position, not sending a goal" << std::endl;
    return tcp_io_device::MsgData::invalidMsgData();
  }

  std::vector<double> mouse_position = { mouse_state.x, mouse_state.y, mouse_state.z };


  double dist_to_center = mathLib::get_distance(mouse_position[0], mouse_position[1]);
  if (dist_to_center > MAX_DIST || dist_to_center < MIN_DIST) {
    // Clicked outside the robot's reachable area.
    std::cout << "WARNING: Clicked outside the robot's reachable area. No goal message is sent." << std::endl;
    return tcp_io_device::MsgData::invalidMsgData();
  }

  std::vector<double> rounded_mouse_position = getRoundedXYZPosition(&(mouse_position[0]));
  std::vector<double> rounded_hand_position = getRoundedXYZPosition(&(hand_xyz_pos_[0]));
#if Z_0
  std::vector<double> delta = { rounded_mouse_position[0] - rounded_hand_position[0], rounded_mouse_position[1] - rounded_hand_position[1], 0.0 };
#else
  std::vector<double> delta = { rounded_mouse_position[0] - rounded_hand_position[0], rounded_mouse_position[1] - rounded_hand_position[1], rounded_mouse_position[2] - rounded_hand_position[2] };
#endif

  std::cout << "Rounded mouse position: " << std::endl;
  std::cout << "x: " << rounded_mouse_position[0] << std::endl;
  std::cout << "y: " << rounded_mouse_position[1] << std::endl;
  std::cout << "z: " << rounded_mouse_position[2] << std::endl;

  std::cout << "Rounded hand position: " << std::endl;
  std::cout << "x: " << rounded_hand_position[0] << std::endl;
  std::cout << "y: " << rounded_hand_position[1] << std::endl;
  std::cout << "z: " << rounded_hand_position[2] << std::endl;

  std::cout << "Delta: " << std::endl;
  std::cout << "x: " << delta[0] << std::endl;
  std::cout << "y: " << delta[1] << std::endl;
  std::cout << "z: " << delta[2] << std::endl;

  for (auto it = commands_meta_data_.begin(); it != commands_meta_data_.end(); ++it) {
    if (id_string_mapping_[it->getEntityID()] == "h" && id_string_mapping_[it->getID()] == "move") {
      tcp_io_device::MsgData goal_proto_var = tcp_io_device::MsgData::createNewMsgData(*it, std::vector<double>{ delta[0], delta[1], delta[2] });
      return goal_proto_var;
    }
  }
  std::cout << "WARNING: No MetaData for move command found. This is some setup issue." << std::endl;
  return tcp_io_device::MsgData::invalidMsgData();
}


void UR3eController::handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) {
  receive_cmd_time_ = running_time_steps_;
  std::cout << "Received message with the following objects:" << std::endl;

  for (auto it = msg_data.begin(); it != msg_data.end(); ++it) {
    std::string id_name = id_string_mapping_[it->getMetaData().getID()];
    std::string entity_name = id_string_mapping_[it->getMetaData().getEntityID()];
    std::cout << "Entity: " + entity_name + " Property: " + id_name << std::endl;

    if (entity_name != "h") {
      // Unknown command
      continue;
    }
    if (id_name == "measure") {
      it->getData<communication_id_t>();
      measurement_state_ = MEASURE_HAND;
    }
    else if (id_name == "grab")
    {
      if (state_ == IDLE || state_ == MOVE_ARM) {
        target_h_position_ = hand_xyz_pos_;
        state_ = MOVE_DOWN_CLOSE;
      }
    }
    else if (id_name == "release")
    {
      if (state_ == IDLE) {
        state_ = MOVE_DOWN_OPEN;
      }
    }
    else if (id_name == "move")
    {
      if (state_ == IDLE || state_ == MOVE_ARM) {
        std::vector<double> move_by_command = it->getData<double>();
        std::cout << "Moving arm by: x: " << move_by_command[0] << ", y: " << move_by_command[1] << ", z: " << move_by_command[2] << ", t: " << move_by_command[3] << std::endl;
        if (move_by_command.size() == 4) {
          move_by_command[0] *= move_by_command[3];
          move_by_command[1] *= move_by_command[3];
          move_by_command[2] *= move_by_command[3];
          move_by_command.pop_back();
        }
        std::cout << "Current arm pos: x: " << hand_xyz_pos_[0] << ", y: " << hand_xyz_pos_[1] << ", z: " << hand_xyz_pos_[2] << std::endl;
        std::cout << "Moving arm by: x: " << move_by_command[0] << ", y: " << move_by_command[1] << ", z: " << move_by_command[2] << std::endl;

        std::vector<double> target_position;
        for (int i = 0; i < move_by_command.size(); ++i) {
          double new_target_position = last_sent_hand_xyz_pos_[i] + move_by_command[i];
          if (fabs(target_h_position_[i] - new_target_position) >= 0.01) {
            target_h_position_[i] = new_target_position;
          }
        }

        std::cout << "Moving arm to: x: " << target_h_position_[0] << ", y: " << target_h_position_[1] << ", z: " << target_h_position_[2] << std::endl;

        state_ = MOVE_ARM;
      }
    }
    else if (id_name == "rotate") {
      std::vector<double> move_by_data = it->getData<double>();
      Eigen::Quaterniond move_by_quaternion = Eigen::Quaterniond(move_by_data[0], move_by_data[1], move_by_data[2], move_by_data[3]);
      move_by_quaternion.normalize();
      //std::cout << "Quaternion:" << std::endl << move_by_quaternion << std::endl;
      auto test = Eigen::AngleAxisd(move_by_quaternion);
      //std::cout << "AngleAxis: Angle: " << std::endl << test.angle() << std::endl << "Axis:" << std::endl << test.axis() << std::endl;
      auto euler_angles = move_by_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
      //std::cout << "EulerAngles: " << std::endl << euler_angles << std::endl;

      //std::cout << "Quaternion calculation: " << std::endl << hand_rotation_ * move_by_quaternion << std::endl;
      target_joint_angles_[5] += euler_angles.y();
      state_ = ROTATE_HAND;
    }
    if (execution_times_map_.find(id_name) != execution_times_map_.end()) {
      max_exec_time_steps_ = execution_times_map_.at(id_name);
    }
    else {
      max_exec_time_steps_ = execution_times_map_.at("default");
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
      std::cout << "ERROR: COULD NOT GET VALID SOLUTION TO MOVE TO (" << xy_pos[0] << ", " << xy_pos[1] << ", " << (0.185 * z_level) + 0.015 << ") TO INVERSE KINEMATIC, NOT MOVING..." << std::endl;
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



std::vector<double> UR3eController::add_gaussian_noise(std::vector<double> values, double std_dev) {

  std::normal_distribution<double> distribution(0.0, std_dev);
  std::vector<double> out;
  for (size_t i = 0; i < values.size(); i++)
  {
    out.push_back(values[i] + distribution(generator_));
  }
  return out;
}