// File:          aera_hand_controller_protobuf.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

#include "hand_grab_sphere_NED_1D_controller.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void run(Robot* robot, int timestep) {
  while (robot->step(timestep) != -1) {}
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

  std::cout << "Starting HandGrabSphereController" << std::endl;
  HandGrabSphereController controller = HandGrabSphereController();
  int err = controller.startConnection();
  if (err != 0) {
    std::cout << "Error when establishing TCP connection. Shutting down..." << std::endl;
    return 1;
  }
  std::cout << "TCP connection successfully established" << std::endl;
  controller.start();

  int timeStep = (int)controller.robot_->getBasicTimeStep();

  std::cout << "TS: " << timeStep << std::endl;

  //run(controller.robot_, timeStep);

  //while (controller->robot_->step(timeStep) != -1) {}

  return 0;
}
