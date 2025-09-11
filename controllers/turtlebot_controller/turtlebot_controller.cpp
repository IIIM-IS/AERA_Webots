#include "turtlebot_controller.h"
#include "../AERA_controller_base/toml_parser.h"

#define DEBUG 1
#define Z_0 1

Turtlebotcontroller::Turtlebotcontroller() : AERAController() {

  state_ = STARTING;
  robot_ = new webots::Supervisor();
  robot_time_step_ = (int)robot_->getBasicTimeStep();
  turtlebot_ = robot_->getFromDef("TurtleBot3Burger");
  motor_left_ = robot_->getMotor("left wheel motor");
  motor_right_ = robot_->getMotor("right wheel motor");
  motor_left_->setPosition(INFINITY);
  motor_right_->setPosition(INFINITY);
  motor_left_->setVelocity(0);
  motor_right_->setVelocity(0);
}

Turtlebotcontroller::~Turtlebotcontroller() {
  if (robot_) {
    delete robot_;
  }
}

int Turtlebotcontroller::start() {

  std::cout << turtlebot_->getDef() << std::endl;

  std::map<std::string, std::map<std::string, tcp_io_device::MetaData> > objects_map;
  objects_map = setup("settings.toml");

  waitForStartMsg();
  init();
  // Step once to initialize joint sensors.
  robot_->step(robot_time_step_);
  std::vector<tcp_io_device::MsgData> data_to_send;

  const double* bot_translation = turtlebot_->getField("translation")->getSFVec3f();
  std::vector<double> bot_position = { bot_translation[0], bot_translation[1], bot_translation[2] };
  std::cout << "Position: (x, y, z)" << std::endl;
  std::cout << "(";
  for (int i = 0; i <= 2; ++i) {
    std::cout << bot_translation[i] << ", ";
  }
  std::cout << std::endl;
  const double* bot_rotation_angle_axis = turtlebot_->getField("rotation")->getSFRotation();
  std::vector<double> bot_rotation = axisAngleToQuaternion(bot_rotation_angle_axis);

  for (auto o = objects_map.begin(); o != objects_map.end(); ++o) {
    std::string entity = o->first;
    for (auto p = o->second.begin(); p != o->second.end(); ++p) {
      std::string prop = p->first;
      if (entity == "bot") {
        if (prop == "position") {
          data_to_send.push_back(createMsgData<double>(p->second, bot_position));
        }
        else if (prop == "rotation") {
          data_to_send.push_back(createMsgData<double>(p->second, bot_rotation));
        }
      }
    }
  }

  sendDataMessage(data_to_send);
  state_ = IDLE;
  run();
  return 0;
}

void Turtlebotcontroller::init() {
  robot_->step(robot_time_step_);
}

void Turtlebotcontroller::run() {

  diagnostic_mode_ = true;
  std::unique_ptr<tcp_io_device::TCPMessage> pending_msg;

  while (robot_->step(robot_time_step_) != -1) {
    if (!aera_started_) {
      std::cout << "AERA not started, wait for start message before calling run()" << std::endl;
      break;
    }
    auto msg = receive_queue_->dequeue();
    if (msg) {
      std::cout << "msg received in controller of type: " << msg->messagetype() << std::endl;
      pending_msg = std::move(msg);
    }
    if (pending_msg) {
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


    const double* bot_translation = turtlebot_->getField("translation")->getSFVec3f();
    std::vector<double> bot_position = add_gaussian_noise({ bot_translation[0], bot_translation[1], bot_translation[2] }, 0.005);

    const double* bot_rotation_angle_axis = turtlebot_->getField("rotation")->getSFRotation();
    std::vector<double> bot_rotation = axisAngleToQuaternion(bot_rotation_angle_axis);


    std::vector<tcp_io_device::MsgData> msg_data;
    for (auto it = objects_meta_data_.begin(); it != objects_meta_data_.end(); ++it) {
      if (id_string_mapping_[it->getID()] == "rotation") {

        std::string entity = id_string_mapping_[it->getEntityID()];
        if (entity == "bot") {
          msg_data.push_back(createMsgData<double>(*it, bot_rotation));
        }
      }
      if (id_string_mapping_[it->getID()] == "position") {
        std::string entity = id_string_mapping_[it->getEntityID()];
        if (entity == "bot") {
          msg_data.push_back(createMsgData<double>(*it, bot_position));
          continue;
        }
      }
    }
    sendDataMessage(msg_data);
    
    executeCommand();
  }
}


void Turtlebotcontroller::executeCommand() {
  std::cout << "Executing command with state_: " << state_ << std::endl;
  switch (state_)
  {
  case STARTING:
  case IDLE:
  case MOVING:
    return;
  default:
    return;
  }

}


void Turtlebotcontroller::handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) {
  std::cout << "Received message with the following objects:" << std::endl;

  for (auto it = msg_data.begin(); it != msg_data.end(); ++it) {
    std::string id_name = id_string_mapping_[it->getMetaData().getID()];
    std::string entity_name = id_string_mapping_[it->getMetaData().getEntityID()];
    std::cout << "Entity: " + entity_name + " Property: " + id_name << std::endl;

    if (entity_name != "bot") {
      // Unknown command
      continue;
    }
    else if (id_name == "move")
    {
      if (state_ == IDLE || state_ == MOVING) {
        std::vector<double> move_velocity = it->getData<double>();

        std::cout << "Received velocity: " << move_velocity[0] << std::endl;

        double vel = move_velocity[0];
        if (vel > 6.67) {
          vel = 6.67;
        }

        double abs_vel = fabs(vel);
        abs_vel = min(abs_vel, 0.1);
        if (vel < 0) {
          vel = abs_vel * -1;
        }
        else {
          vel = abs_vel;
        }

        std::cout << "Moving with velocity: " << vel << std::endl;
        double max_force =  motor_left_->getMaxForce();
        double max_torque = motor_left_->getMaxTorque();
        std::cout << "Max Force: " << max_force << std::endl;
        std::cout << "Max Torque: " << max_torque << std::endl;
        // motor_left_->setVelocity(vel);
        // motor_right_->setVelocity(vel);
        motor_left_->setTorque(vel);
        motor_right_->setTorque(vel);

      }
    }
    else if (id_name == "rotate")
    {
      if (state_ == IDLE || state_ == MOVING) {
        std::vector<double> rotate_velocity = it->getData<double>();

        std::cout << "Rotating with velocity: " << rotate_velocity[0] << std::endl;

        if (rotate_velocity[0] < 0) {
          motor_left_->setVelocity(-rotate_velocity[0]);
          motor_right_->setVelocity(rotate_velocity[0]);
        }
        else {
          motor_left_->setVelocity(rotate_velocity[0]);
          motor_right_->setVelocity(-rotate_velocity[0]);
        }

      }
    }
  }
}



std::vector<double> Turtlebotcontroller::add_gaussian_noise(std::vector<double> values, double std_dev) {

  std::normal_distribution<double> distribution(0.0, std_dev);
  std::vector<double> out;
  for (size_t i = 0; i < values.size(); i++)
  {
    out.push_back(values[i] + distribution(generator_));
  }
  return out;
}