#pragma once
#include "aera_controller.h"
#include "universal_robots_kinematics.h"
#include "tip_camera.h"
#include <random>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "../../submodules/Eigen/Eigen/Geometry"

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

  webots::InertialUnit* inertial_unit_;

  TipCamera* tip_camera_;

  webots::Node* objects_[NUMBER_OF_BOXES];

  universalRobots::UR* ur_kinematics_;

  int robot_time_step_;

  int running_time_steps_;
  int receive_cmd_time_;

  int max_exec_time_steps_;

  const std::map<std::string, int> execution_times_map_ = { {"move", 10'000}, {"grab", 1'500}, {"release", 1'500}, {"default", 10'000} };

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

  const double motor_offsets_[NUMBER_OF_ARM_MOTORS] = { 0.0, -M_PI / 2, 0.0, -M_PI / 2, 0.0, 0.0 };

  const double box_positions_[NUMBER_OF_BOXES][3] = { {0.0, 1.0, 0.05}, {0.2, 0.4, 0.05}, {-0.5, 0.3, 0.05} };
  const double box_rotations_[NUMBER_OF_BOXES][4] = { {0.0, 0.0, 1.0, 0}, {0.0, 0.0, 1.0, 0}, {0.0, 0.0, 1.0, 0} };

private:

  enum State {
    STARTING,
    IDLE,
    STOPPING,
    MOVE_ARM,
    OPEN_BEFORE_GRAB_2,
    MOVE_DOWN_CLOSE_1,
    MOVE_DOWN_CLOSE_2,
    MOVE_DOWN_OPEN,
    OPEN,
    CLOSE_GRAB_1,
    CLOSE_GRAB_2,
    MOVE_UP,
    MOVE_UP_AFTER_RELEASE,
    HAND_TO_DEFAULT
  };

  State state_;

  std::vector<double> hand_xyz_pos_;
  std::vector<double> target_h_position_;
  std::vector<double> target_joint_angles_;
  std::vector<double> hand_default_values_;
  std::vector<double> hand_open_values_;
  std::vector<double> hand_closed_1_values_;
  std::vector<double> hand_closed_2_values_;

  Eigen::Quaterniond hand_rotation_;
  Eigen::Quaterniond target_hand_rotation_;

  const double position_accuracy_error_ = 0.01;
  const double gripper_accuracy_error_ = 0.05;

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

  static std::vector<double> getRoundedXYPosition(const double* pos_array, int decimal_places = 1) {
    std::vector<double> out;
    out.push_back(round(pos_array[0] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[1] * pow(10., decimal_places)) / pow(10., decimal_places));
    return out;
  }

  static std::vector<double> getRoundedXYZPosition(const double* pos_array, int decimal_places = 1) {
    std::vector<double> out;
    out.push_back(round(pos_array[0] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[1] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[2] * pow(10., decimal_places)) / pow(10., decimal_places));
    return out;
  }
};