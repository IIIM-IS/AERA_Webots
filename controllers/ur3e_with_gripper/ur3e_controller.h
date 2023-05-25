#pragma once
#include "aera_controller.h"
#include "universal_robots_kinematics.h"
#include "webots/Camera.hpp"
#include <random>

#define NUMBER_OF_ARM_MOTORS 6
#define NUMBER_OF_HAND_MOTORS 3
#define NUMBER_OF_KINEMATIC_SOLUTIONS 8
#define SOLUTION_TO_CHOOSE 3

#define NUMBER_OF_BOXES 3
#define MIN_DIST 0.4
#define MAX_DIST 1.3


class UR3eController :
  public AERAController
{
public:
  UR3eController();
  ~UR3eController();

  int start() override;

  void init();

  void run() override;

protected:

  webots::Node* ur3e_robot_arm_;

  webots::Motor* arm_motors_[NUMBER_OF_ARM_MOTORS];
  webots::Motor* hand_motors_[NUMBER_OF_HAND_MOTORS];

  webots::PositionSensor* arm_sensors_[NUMBER_OF_ARM_MOTORS];
  webots::PositionSensor* hand_sensors_[NUMBER_OF_HAND_MOTORS];

  webots::GPS* gps_sensor_;

  webots::Camera* tip_camera_;

  webots::Node* boxes_[NUMBER_OF_BOXES];

  universalRobots::UR* ur_kinematics_;
  int robot_time_step_;

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

  const double motor_offsets_[NUMBER_OF_ARM_MOTORS] = { 0.0, -M_PI / 2, 0.0, -M_PI / 2, 0.0, 0.0 };

  const double box_positions_[NUMBER_OF_BOXES][3] = { {0.4, 0.2, 0.05}, {0.2, 0.4, 0.05}, {-0.5, 0.3, 0.05} };
  const double box_rotations_[NUMBER_OF_BOXES][4] = { {0.0, 0.0, 1.0, 0}, {0.0, 0.0, 1.0, 0}, {0.0, 0.0, 1.0, 0} };

private:

  enum State { STARTING, IDLE, MOVE_DOWN_CLOSE, MOVE_DOWN_OPEN, MOVE_UP, MOVE_ARM, CLOSE_GRIPPER, OPEN_GRIPPER, STOPPING };

  State state_;

  std::vector<double> target_h_position_;
  std::vector<double> target_joint_angles_;
  std::vector<double> hand_closed_values_;
  std::vector<double> hand_open_values_;

  const double position_accuracy_error_ = 0.001;
  const double gripper_accuracy_error_ = 0.0005;

  void executeCommand();

  std::vector<double> getJointAnglesFromXY(std::vector<double> xy_pos, int z_level);

  void setJointAngles(std::vector<double> joint_angles);

  void setHandAngles(std::vector<double> hand_values);

  static void get_random_box_positions(double positions[NUMBER_OF_BOXES][3]) {
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::default_random_engine gen(time);
    std::uniform_real_distribution<double> distrib(-MAX_DIST, MAX_DIST);


    for (unsigned int i = 0; i < NUMBER_OF_BOXES; i++)
    {
      double x;
      double y;
      while (true) {
        x = round(distrib(gen) * 100.) / 100.;
        y = round(distrib(gen) * 100.) / 100.;
        double dist_to_center = mathLib::get_distance(x, y);
        if (dist_to_center > MAX_DIST || dist_to_center < MIN_DIST) {
          continue;
        }
        if (i == 0) {
          break;
        }
        bool redraw = false;
        for (int j = i; j > 0; --j) {
          double dist_to_other = mathLib::get_distance(x, y, positions[j - 1][0], positions[j - 1][1]);
          redraw = redraw || dist_to_other < MIN_DIST;
        }
        if (!redraw) {
          break;
        }
      }
      positions[i][0] = x;
      positions[i][1] = y;
      positions[i][2] = 0.05;
    }
  }

  static std::vector<double> getRoundedXYPosition(const double* pos_array, int decimal_places = 2) {
    std::vector<double> out;
    out.push_back(round(pos_array[0] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[1] * pow(10., decimal_places)) / pow(10., decimal_places));
    return out;
  }
};