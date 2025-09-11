#pragma once
#include "../AERA_controller_base/aera_controller.h"
#include "tip_camera.h"
#include <random>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "../../submodules/Eigen/Eigen/Geometry"


class Turtlebotcontroller :
  public AERAController
{
public:
  Turtlebotcontroller();
  ~Turtlebotcontroller();

  int start() override;

  void init();

  void run() override;

protected:

  webots::Node* turtlebot_;

  void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) override;

private:

  enum State { STARTING, IDLE, MOVING };

  State state_;

  webots::Motor* motor_left_;
  webots::Motor* motor_right_;

  std::vector<double> linear_velocity_;
  double target_acceleration_;
  // double rotation_velocity_;

  std::default_random_engine generator_;

  void executeCommand();

  std::vector<double> add_gaussian_noise(std::vector<double> values, double std_dev);

  static std::vector<double> getRoundedXYPosition(const double* pos_array, int decimal_places = 2) {
    std::vector<double> out;
    out.push_back(round(pos_array[0] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[1] * pow(10., decimal_places)) / pow(10., decimal_places));
    return out;
  }

  static std::vector<double> getRoundedXYZPosition(const double* pos_array, int decimal_places = 2) {
    std::vector<double> out;
    out.push_back(round(pos_array[0] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[1] * pow(10., decimal_places)) / pow(10., decimal_places));
    out.push_back(round(pos_array[2] * pow(10., decimal_places)) / pow(10., decimal_places));
    return out;
  }

  static std::vector<double> add_gaussian_noise(std::vector<double> values, std::vector<double> std_devs) {
    if (values.size() != std_devs.size()) {
      return values;
    }

    std::default_random_engine generator;
    std::vector<double> out;
    for (size_t i = 0; i < values.size(); i++)
    {
      std::normal_distribution<double> distribution(values[i], std_devs[i]);
      out.push_back(distribution(generator));
    }
    return out;
  }

  static std::vector<double> axisAngleToQuaternion(const double* axis_angle) {
    const double& nx = axis_angle[0];
    const double& ny = axis_angle[1];
    const double& nz = axis_angle[2];
    const double& angle = axis_angle[3];

    double half_angle = angle * 0.5;
    double sin_half = std::sin(half_angle);
    double cos_half = std::cos(half_angle);

    double w = cos_half;
    double x = nx * sin_half;
    double y = ny * sin_half;
    double z = nz * sin_half;

    if (w < 0.0) {
      w = -w;
      x = -x;
      y = -y;
      z = -z;
    }
    return { w, x, y, z };
  }
};