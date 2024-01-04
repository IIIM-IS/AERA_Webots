#include "ur3e_controller.h"
#include "toml_parser.h"
#include <cmath>

#define DEBUG 1
#define Z_0 1

UR3eController::UR3eController() : AERAController() {

  state_ = STARTING;
  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  running_time_steps_ = 0;
  max_exec_time_steps_ = execution_times_map_.at("default");
  receive_cmd_time_ = INT_MAX - 1;
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

  objects_[0] = robot_->getFromDef("small_cube");
  objects_[1] = robot_->getFromDef("large_cube");
  objects_[2] = robot_->getFromDef("large_sphere");
  objects_[3] = robot_->getFromDef("small_sphere");
  objects_[4] = robot_->getFromDef("large_cylinder");


  gps_sensor_ = robot_->getGPS("gps_tip");

  inertial_unit_ = robot_->getInertialUnit("inertial unit");

  tip_camera_ = new TipCamera(robot_->getCamera("tip_camera"));

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

  inertial_unit_->enable(robot_time_step_);

  tip_camera_->enable(robot_time_step_);

  hand_closed_2_values_ = { 0.5, 0.5, 0.5 };
  hand_closed_1_values_ = { 0.6, 0.6, 0.6 };
  for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
    double center = hand_motors_[i]->getMaxPosition() - hand_motors_[i]->getMinPosition();
    //hand_default_values_.push_back(center);
    hand_default_values_.push_back(0.45);
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

  toml_parser::TOMLParser parser;
  parser.parse("settings.toml");

  std::vector<tcp_io_device::MetaData> objects;
  std::vector<tcp_io_device::MetaData> commands;


  std::vector<std::string> entity_names = parser.entityNames();
  std::vector<std::string> property_names = parser.propertyNames();
  std::vector<std::string> command_names = parser.commandNames();

  std::vector<std::string> object_names;
  object_names.resize(entity_names.size() + property_names.size() + command_names.size());
  // Add entities to the vector.
  object_names.insert(object_names.end(), entity_names.begin(), entity_names.end());
  // Add properties to the vector
  object_names.insert(object_names.end(), property_names.begin(), property_names.end());
  // Add commands to the vector
  object_names.insert(object_names.end(), command_names.begin(), command_names.end());

  // Generate communication ids by filling the string_id_mapping_
  fillIdStringMaps(object_names);

  std::map<std::string, toml_parser::entity> entity_map = parser.entities();
  for (auto e_it = entity_map.begin(); e_it != entity_map.end(); ++e_it) {
    auto e = e_it->second;
    for (auto p_it = e.properties.begin(); p_it != e.properties.end(); ++p_it) {
      objects.push_back(tcp_io_device::MetaData(string_id_mapping_[e_it->first], string_id_mapping_[p_it->name], p_it->data_type, p_it->dimensions, p_it->opcode_handle));
    }
    for (auto c_it = e.commands.begin(); c_it != e.commands.end(); ++c_it) {
      commands.push_back(tcp_io_device::MetaData(string_id_mapping_[e_it->first], string_id_mapping_[c_it->name], c_it->data_type, c_it->dimensions, c_it->opcode_handle));
    }
  }

  sendSetupMessage(objects, commands);
  waitForStartMsg();
  init();
  // Step once to initialize joint sensors.
  robot_->step(robot_time_step_);
  running_time_steps_ += robot_time_step_;

  const double* hand_pos_array = gps_sensor_->getValues();
  hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);
  
  std::vector<std::vector<double>> obj_xyz_positions;
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    const double* obj_pos_array = objects_[i]->getField("translation")->getSFVec3f();
    std::vector<double> box_xyz_pos = getRoundedXYZPosition(obj_pos_array);
    obj_xyz_positions.push_back(box_xyz_pos);
  }

  // get the orientations of hand and objects
  const double* hand_ori_array = inertial_unit_->getQuaternion();
  hand_quaternion_ = getRoundedOrientation(hand_ori_array); // WARNING: the function may need modifications

  std::vector<std::vector<double>> obj_wxyz_orientations;
  for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
    const double* obj_ori_array = objects_[i]->getField("rotation")->getSFRotation();
    std::vector<double> box_orientation = getRoundedOrientation(obj_ori_array);
    obj_wxyz_orientations.push_back(box_orientation);
  }

  std::vector<tcp_io_device::MsgData> data_to_send;
  for (auto o = objects.begin(); o != objects.end(); ++o) {
    std::string entity = id_string_mapping_[o->getEntityID()];
    std::string prop = id_string_mapping_[o->getID()];
    if (entity == "h") {
      if (prop == "position") {
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { hand_xyz_pos_[0], hand_xyz_pos_[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, hand_xyz_pos_));
#endif
      }
      else if (prop == "orientation") {
        // has no effect at all
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ hand_quaternion_[0], hand_quaternion_[1], hand_quaternion_[2], hand_quaternion_[3]})));
      }
      else if (prop == "holding") {
        data_to_send.push_back(createMsgData<communication_id_t>(*o, { -1 }));
      }
    }
    else if (entity == "cube_1") {
      if (prop == "position") {
        std::vector<double> box_pos = obj_xyz_positions[0];
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { box_pos[0], box_pos[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_pos));
#endif
      }
      else if (prop == "orientation") {
        std::vector<double> box_ori = obj_wxyz_orientations[0];
        // send the orientation data
#if Z_0        
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ box_ori[0], box_ori[1], box_ori[2], box_ori[3] })));
        // CHECK THE CURRENT ORIENTATION OF THE FIRST CUBE
        std::cout << "box_ori[1] " << box_ori[0] << ", qx: " << box_ori[1] << ", qy: " << box_ori[2] << ", qz: " << box_ori[3] << std::endl;
#else
        data_to_send.push_back(createMsgData<double>(*o, box_ori));
#endif
      }
    }
    else if (entity == "cube_2") {
      if (prop == "position") {
        std::vector<double> box_pos = obj_xyz_positions[1];
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { box_pos[0], box_pos[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_pos));
#endif
      }
      else if (prop == "orientation") {
        std::vector<double> box_ori = obj_wxyz_orientations[1];
        // send the orientation data
#if Z_0        
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ box_ori[0], box_ori[1], box_ori[2], box_ori[3] })));
        // CHECK THE CURRENT ORIENTATION OF THE second CUBE
        std::cout << "box_ori[2] " << box_ori[0] << ", qx: " << box_ori[1] << ", qy: " << box_ori[2] << ", qz: " << box_ori[3] << std::endl;
#else
        data_to_send.push_back(createMsgData<double>(*o, box_ori));
#endif
      }
    }
    else if (entity == "sphere_1") {
      if (prop == "position") {
        std::vector<double> box_pos = obj_xyz_positions[2];
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { box_pos[0], box_pos[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_pos));
#endif
      }
      else if (prop == "orientation") {
        std::vector<double> box_ori = obj_wxyz_orientations[2];
        // send the orientation data
#if Z_0        
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ box_ori[0], box_ori[1],box_ori[2], box_ori[3] })));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_ori));
#endif
      }
    }
    else if (entity == "sphere_2") {
      if (prop == "position") {
        std::vector<double> box_pos = obj_xyz_positions[3];
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { box_pos[0], box_pos[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_pos));
#endif
      }
      else if (prop == "orientation") {
        std::vector<double> box_ori = obj_wxyz_orientations[3];
        // send the orientation data
#if Z_0        
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ box_ori[0], box_ori[1], box_ori[2], box_ori[3] })));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_ori));
#endif
      }
    }
    else if (entity == "cyl_1") {
      if (prop == "position") {
        std::vector<double> box_pos = obj_xyz_positions[4];
#if Z_0
        data_to_send.push_back(createMsgData<double>(*o, { box_pos[0], box_pos[1], 0. }));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_pos));
#endif
      }
      else if (prop == "orientation") {
        std::vector<double> box_ori = obj_wxyz_orientations[4];
        // send the orientation data
#if Z_0        
        data_to_send.push_back(createMsgData<double>(*o, std::vector<double>({ box_ori[0], box_ori[1], box_ori[2], box_ori[3] })));
#else
        data_to_send.push_back(createMsgData<double>(*o, box_ori));
#endif
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

  for (size_t i = 0; i < NUMBER_OF_HAND_MOTORS; ++i)
  {
    hand_motors_[i]->setPosition(hand_default_values_[i]);
  }
  double random_positions[NUMBER_OF_BOXES][3];

  // get_random_box_positions is not needed, as the positions and orientations are not random.
  //get_random_box_positions(random_positions);
#if 1 // debug
  random_positions[0][0] = 0;
  random_positions[0][1] = 1;
  random_positions[1][0] = -0.5;
  random_positions[1][1] = 1;
  random_positions[2][0] = -.8;
  random_positions[2][1] = -0.1;
  random_positions[3][0] = -1.1;
  random_positions[3][1] = -0.4;
  random_positions[4][0] = -1;
  random_positions[4][1] = -0.71;
#endif

  double orientations[NUMBER_OF_BOXES][4];
  // Note that [0, 0, 0, 0] gets error, as it is in quaternions
  // Since the axis of rotation os z axis, we can assume qx =0, qy=0.
  orientations[0][0] = 1;
  orientations[0][1] = 0;
  orientations[0][2] = 0;
  orientations[0][3] = 0;

  orientations[1][0] = 1;
  orientations[1][1] = 0;
  orientations[1][2] = 0;
  orientations[1][3] = 0;

  orientations[2][0] = 1;
  orientations[2][1] = 0;
  orientations[2][2] = 0;
  orientations[2][3] = 0;

  orientations[3][0] = 1;
  orientations[3][1] = 0;
  orientations[3][2] = 0;
  orientations[3][3] = 0;

  orientations[4][0] = 1;
  orientations[4][1] = 0;
  orientations[4][2] = 0;
  orientations[4][3] = 0;

  for (size_t i = 0; i < NUMBER_OF_BOXES; i++)
  {
    objects_[i]->getField("rotation")->setSFRotation(orientations[i]);
    objects_[i]->getField("translation")->setSFVec3f(random_positions[i]);
  }

#if 0
  boxes_[0]->getField("rotation")->setSFRotation(orientations[0]);
  boxes_[0]->getField("translation")->setSFVec3f(box_positions_[0]);
#endif
}

void UR3eController::run() {

  int aera_us = 100'000 / robot_time_step_;
#ifdef DEBUG
  diagnostic_mode_ = true;
#endif // DEBUG
  int receive_deadline = aera_us + 65'000 / robot_time_step_;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    if (msg) {
      pending_msg = std::move(msg);
    }
    if (pending_msg && (!diagnostic_mode_ || aera_us == receive_deadline)) {
      if (pending_msg->messagetype() == tcp_io_device::TCPMessage_Type_DATA) {
        handleDataMsg(dataMsgToMsgData(std::move(pending_msg)));
        std::cout << "HandleDataMsg called" << std::endl;
        robot_->step(1);
      }
      else {
        std::cout << "Received message with unexpected type "
          << tcp_io_device::TCPConnection::type_to_name_map_[pending_msg->messagetype()]
          << ". Ignoring the message..." << std::endl;
      }
      pending_msg = NULL;
    }
    // Don't send the state on the first pass, but wait to arrive at the initial position.
    if (aera_us > 100'000 / robot_time_step_ && (aera_us % (int)(100'000 / robot_time_step_)) == 0) {

      const double* hand_pos_array = gps_sensor_->getValues();
      hand_xyz_pos_ = getRoundedXYZPosition(hand_pos_array);

      // get the orientations of hand and objects
      const double* hand_ori_array = inertial_unit_->getQuaternion();
      hand_quaternion_ = getRoundedOrientation(hand_ori_array); 

      // This piece of code had no use.
      //const double* hand_rotation = inertial_unit_->getQuaternion();
      //double _1 = hand_rotation[3] < 0 ? -1 : 1;
      //Eigen::Quaterniond new_rotation = Eigen::Quaterniond(
      //  _1 * round(hand_rotation[3] * 100.) / 100.,
      //  _1 * round(hand_rotation[0] * 100.) / 100.,
      //  _1 * round(hand_rotation[1] * 100.) / 100.,
      //  _1 * round(hand_rotation[2] * 100.) / 100.);
      //new_rotation.normalize();

      //std::cout << "new_rotation: " << new_rotation << std::endl;
      //std::cout << "old_rotation: " << hand_rotation_ << std::endl;

      //if (!(new_rotation.coeffs() == (hand_rotation_.coeffs() * -1.) || new_rotation == hand_rotation_)) {
      //  hand_rotation_ = new_rotation;
      //}

      //std::cout << "old_rotation: " << hand_rotation_ << std::endl;

      std::vector<std::vector<double>> box_xyz_positions;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        const double* box_pos_array = objects_[i]->getField("translation")->getSFVec3f();
        std::vector<double> box_xyz_pos = getRoundedXYZPosition(box_pos_array);
        box_xyz_positions.push_back(box_xyz_pos);
      }

      std::vector<std::vector<double>> box_wxyz_orientations;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        const double* box_ori_array = objects_[i]->getField("rotation")->getSFRotation();
        std::vector<double> box_wxyz_ori = getRoundedXYZPosition(box_ori_array);
        box_wxyz_orientations.push_back(box_wxyz_ori);
      }

      communication_id_t holding_id = -1;
      for (int i = 0; i < NUMBER_OF_BOXES; ++i) {
        // hand must be at the same place as object to grab
        if (sqrt((hand_xyz_pos_[0] - box_xyz_positions[i][0]) * (hand_xyz_pos_[0] - box_xyz_positions[i][0]) +
          (hand_xyz_pos_[1] - box_xyz_positions[i][1]) * (hand_xyz_pos_[1] - box_xyz_positions[i][1])) > 0.1) {
          continue;
        }
        // if the hand is holding an object, its altitute changes.
        if (box_xyz_positions[i][2] > 0.014)
        {
          if (i == 0)
            holding_id = string_id_mapping_["cube_1"];
          else if (i == 1)
            holding_id = string_id_mapping_["cube_2"];
          else if (i == 2)
            holding_id = string_id_mapping_["sphere_1"];
          else if (i == 3)
            holding_id = string_id_mapping_["sphere_2"];
          else if (i == 4)
            holding_id = string_id_mapping_["cyl_1"];
          break;
        }
      }

      tip_camera_->recognitionEnable(1);
      robot_->step(1);

      std::vector<tcp_io_device::MsgData> msg_data;
      for (auto it = objects_meta_data_.begin(); it != objects_meta_data_.end(); ++it) {
        if (id_string_mapping_[it->getID()] == "holding") {
          msg_data.push_back(createMsgData<communication_id_t>(*it, { holding_id }));
          continue;
        }
        if (id_string_mapping_[it->getID()] == "position") {
          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity == "h") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = hand_xyz_pos_[2];
            hand_xyz_pos_[2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, hand_xyz_pos_));
#if Z_0
            hand_xyz_pos_[2] = save_z;
#endif
            //measurement_state_ = NONE;
            continue;
          }
          else if (entity == "cube_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[0][2];
            box_xyz_positions[0][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[0]));
#if Z_0
            box_xyz_positions[0][2] = save_z;
#endif
            continue;
          }
          else if (entity == "cube_2") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[1][2];
            box_xyz_positions[1][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[1]));
#if Z_0
            box_xyz_positions[1][2] = save_z;
#endif
            continue;
          }
          else if (entity == "sphere_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[2][2];
            box_xyz_positions[2][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[2]));
#if Z_0
            box_xyz_positions[2][2] = save_z;
#endif
            continue;
          }
          else if (entity == "sphere_2") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[3][2];
            box_xyz_positions[3][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[3]));
#if Z_0
            box_xyz_positions[3][2] = save_z;
#endif
            continue;
          }
          else if (entity == "cyl_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            double save_z = box_xyz_positions[4][2];
            box_xyz_positions[4][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_xyz_positions[4]));
#if Z_0
            box_xyz_positions[4][2] = save_z;
#endif
            continue;
          }
        }
        if (id_string_mapping_[it->getID()] == "orientation") {
          std::string entity = id_string_mapping_[it->getEntityID()];
          if (entity == "h") {
#if Z_0 // Debug: 
            //double save_z = hand_quaternion_[2];
            //hand_quaternion_[2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, hand_quaternion_));
#if Z_0
            //hand_quaternion_[2] = save_z;
#endif
            //measurement_state_ = NONE;
            continue;
          }
          else if (entity == "cube_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            //double save_z = box_xyz_positions[0][2];
            //box_xyz_positions[0][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_wxyz_orientations[0]));
#if Z_0
            //box_xyz_positions[0][2] = save_z;
#endif
            continue;
          }
          else if (entity == "cube_2") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            //double save_z = box_xyz_positions[1][2];
            //box_xyz_positions[1][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_wxyz_orientations[1]));
#if Z_0
            //box_xyz_positions[1][2] = save_z;
#endif
            continue;
          }
          else if (entity == "sphere_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            //double save_z = box_xyz_positions[2][2];
            //box_xyz_positions[2][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_wxyz_orientations[2]));
#if Z_0
            //box_xyz_positions[2][2] = save_z;
#endif
            continue;
          }
          else if (entity == "sphere_2") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            //double save_z = box_xyz_positions[3][2];
            //box_xyz_positions[3][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_wxyz_orientations[3]));
#if Z_0
            //box_xyz_positions[3][2] = save_z;
#endif
            continue;
          }
          else if (entity == "cyl_1") {
#if Z_0 // Debug: Always report Z = 0 so that hand and box can be at the "same" vec3.
            //double save_z = box_xyz_positions[4][2];
            //box_xyz_positions[4][2] = 0;
#endif
            msg_data.push_back(createMsgData<double>(*it, box_wxyz_orientations[4]));
#if Z_0
            //box_xyz_positions[4][2] = save_z;
#endif
            continue;
          }
        }
      }
      tip_camera_->recognitionDisable();
      // std::cout << "Sending message with following data:" << std::endl;
      sendDataMessage(msg_data);
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
  case UR3eController::STARTING:
  case UR3eController::IDLE:
  case UR3eController::STOPPING:
    break;
  case UR3eController::MOVE_ARM:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = IDLE;
      return;
    }
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 1);
    //target_joint_angles_2_ = setHandOrientation(target_h_position_, target_h_orientation_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_ARM;
        break;
      }
      state_ = IDLE;
    }
  case UR3eController::ROTATE_ARM:
    //if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
    //  state_ = IDLE;
    //  return;
    //}
    // Keep the hand's xy position but change the hands orientation
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > orientation_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = ROTATE_ARM;
        break;
      }
      state_ = IDLE;
    }
    return;
  case UR3eController::OPEN_BEFORE_GRAB_2:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_open_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_open_values_);
        // This is where the had will rotate
        //setJointAngles(target_joint_angles_2_);
        state_ = OPEN_BEFORE_GRAB_2;
        break;
      }
      hand_state_ = HAND_OPEN_2;
      state_ = MOVE_DOWN_CLOSE_2;
    }
    return;
  case UR3eController::MOVE_DOWN_CLOSE_1:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = MOVE_UP;
      return;
    }
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 0);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_CLOSE_1;
        break;
      }
      state_ = CLOSE_GRAB_1;
    }
    return;
  case UR3eController::MOVE_DOWN_CLOSE_2:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = MOVE_UP;
      return;
    }
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 0);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_CLOSE_2;
        break;
      }
      state_ = CLOSE_GRAB_2;
    }
    return;
  case UR3eController::MOVE_DOWN_OPEN:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = MOVE_UP;
      return;
    }
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 0);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_DOWN_OPEN;
        break;
      }
      if (hand_state_ == HAND_CLOSED_1)
        state_ = OPEN_1;
      else if (hand_state_ == HAND_CLOSED_2)
        state_ = OPEN_2;
      else
        state_ = OPEN_1;
    }
    return;
  case UR3eController::OPEN_1:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_default_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_default_values_);
        state_ = OPEN_1;
        break;
      }
      hand_state_ = HAND_OPEN_1;
      state_ = MOVE_UP_AFTER_RELEASE;
    }
    return;
  case UR3eController::OPEN_2:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_open_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_open_values_);
        state_ = OPEN_2;
        break;
      }
      hand_state_ = HAND_OPEN_2;
      state_ = MOVE_UP_AFTER_RELEASE;
    }
    return;
  case UR3eController::CLOSE_GRAB_1:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = OPEN_1;
      return;
    }
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_closed_1_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_closed_1_values_);
        state_ = CLOSE_GRAB_1;
        break;
      }
      hand_state_ = HAND_CLOSED_1;
      state_ = MOVE_UP;
    }
    return;
  case UR3eController::CLOSE_GRAB_2:
    if ((running_time_steps_ - receive_cmd_time_) >= max_exec_time_steps_) {
      state_ = OPEN_2;
      return;
    }
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_closed_2_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_closed_2_values_);
        state_ = CLOSE_GRAB_2;
        break;
      }
      hand_state_ = HAND_CLOSED_2;
      state_ = MOVE_UP;
    }
    return;
  case UR3eController::MOVE_UP:
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_UP;
        break;
      }
      if (hand_state_ == HAND_OPEN_1)
        state_ = HAND_TO_DEFAULT;
      else
        state_ = IDLE;
    }
    return;
  case UR3eController::MOVE_UP_AFTER_RELEASE:
    target_joint_angles_ = getJointAnglesFromXYOrientation(target_h_position_, target_h_orientation_, 1);
    for (int i = 0; i < NUMBER_OF_ARM_MOTORS; ++i) {
      if (fabs(target_joint_angles_[i] - arm_sensors_[i]->getValue()) > position_accuracy_error_) {
        setJointAngles(target_joint_angles_);
        state_ = MOVE_UP_AFTER_RELEASE;
        break;
      }
      if (hand_state_ == HAND_OPEN_2)
        state_ = HAND_TO_DEFAULT;
      else
        state_ = IDLE;
    }
    return;
  case UR3eController::HAND_TO_DEFAULT:
    for (int i = 0; i < NUMBER_OF_HAND_MOTORS; ++i) {
      if (fabs(hand_default_values_[i] - hand_sensors_[i]->getValue()) > gripper_accuracy_error_) {
        setHandAngles(hand_default_values_);
        state_ = HAND_TO_DEFAULT;
        break;
      }
      hand_state_ = HAND_OPEN_1;
      state_ = IDLE;
    }
    return;
  default:
    break;
  }
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
    else if (id_name == "grab_type1")
    {
      state_ = MOVE_DOWN_CLOSE_1;
    }
    else if (id_name == "grab_type2")
    {
      state_ = OPEN_BEFORE_GRAB_2;
    }
    else if (id_name == "release")
    {
      state_ = MOVE_DOWN_OPEN;
    }
    else if (id_name == "move")
    {
      std::vector<double> move_by_command = it->getData<double>();

      std::vector<double> target_position;
      // P1 = P0 + DeltaP
      for (int i = 0; i < move_by_command.size(); ++i) {
        target_position.push_back(hand_xyz_pos_[i] + move_by_command[i]);
      }
      
      target_h_position_ = target_position;

      std::cout << "Moving arm to: x: " << target_h_position_[0] << ", y: " << target_h_position_[1] << std::endl;
      
      state_ = MOVE_ARM;
    }
    else if (id_name == "rotate")
    {
      std::vector<double> rotate_by_command = it->getData<double>();

      std::vector<double> target_orientation;
      // P1 = P0 + DeltaP
      for (int i = 0; i < rotate_by_command.size(); ++i) {
        target_orientation.push_back(hand_quaternion_[i] + rotate_by_command[i]);
      }

      // CHECK THE CURRENT ORIENTATION OF THE FIRST CUBE: wired that it does not matche the sent data!
      const double* obj_ori_array = objects_[1]->getField("rotation")->getSFRotation();
      std::vector<double> box_orientation = getRoundedOrientation(obj_ori_array);
      std::cout << "1st box current orientation qw: " << box_orientation[0] << ", qx: " << box_orientation[1] << ", qy: " << box_orientation[2] << ", qz: " << box_orientation[3] << std::endl;

      /// CHECK THE CURRENT ORIENTATION OF HAND
      const double* hand_orien = inertial_unit_->getQuaternion();
      hand_quaternion_ = getRoundedOrientation(hand_orien); // WARNING: the function may need modifications
      std::cout << "Arm current orientation qw: " << hand_quaternion_[0] << ", qx: " << hand_quaternion_[1] << ", qy: " << hand_quaternion_[2] << ", qz: " << hand_quaternion_[3] << std::endl;

      target_h_orientation_ = target_orientation;
      std::cout << "Rotating the arm to qw: " << target_h_orientation_[0] << ", qx: " << target_h_orientation_[1] << ", qy: " << target_h_orientation_[2]  << ", qz: " << target_h_orientation_[3] << std::endl;

      state_ = ROTATE_ARM;
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
      std::cout << "ERROR: COULD NOT GET VALID SOLUTION TO INVERSE KINEMATIC, NOT MOVING..." << std::endl;
      return std::vector<double>();
    }
    out[i] += motor_offsets_[i];
  }
  return out;
}

// This function sets the orinetation of the hand
std::vector<double> UR3eController::getJointAnglesFromXYOrientation(std::vector<double> xy_pos, std::vector<double> wz_quater, int z_level) {

  float joint_angles_array[8][6];

  // These are conversion formulas from the hand's quaternion angles to Euler angles.
  float phi = atan2(2 * ((wz_quater[0] * wz_quater[1]) + (wz_quater[0] * wz_quater[1])) , (1 - (2*(pow(wz_quater[1], 2) + pow(wz_quater[2], 2)) ) ));
  float theta = asin(2 * ((wz_quater[1]* wz_quater[2]) - (wz_quater[2] * wz_quater[4])));
  float sai = atan2(2 * ((wz_quater[0] * wz_quater[3]) + (wz_quater[1] * wz_quater[2])) , (1 - (2*(pow(wz_quater[2], 2) + pow(wz_quater[3], 2)) ) ));
  
  //double sai = wz_quater[3];
  //std::cout << "SAI IS" << sai << std::endl;

  ur_kinematics_->inverseKinematics(universalRobots::pose(xy_pos[0], xy_pos[1], (0.185 * z_level) + 0.015, M_PI, 0.0, sai), &joint_angles_array);

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
