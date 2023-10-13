#pragma once

#include <iostream>
#include <vector>
#include "webots/Camera.hpp"

class TipCamera : public webots::Camera
{
public:
  TipCamera(webots::Camera* cam) : webots::Camera(*cam) {};

  std::string getColorAsString() {
    int width = getWidth();
    int height = getHeight();
    int size = width = height;
    int byte_size = size * 4;
    const unsigned char* img = getImage();
    int sum_b = 0;
    int sum_g = 0;
    int sum_r = 0;
    for (int pos = 0; pos < byte_size; pos += 4) {
      sum_b += (int)img[pos];
      sum_g += (int)img[pos + 1];
      sum_r += (int)img[pos + 2];
    }
    int mean_b = sum_b / size;
    int mean_g = sum_g / size;
    int mean_r = sum_r / size;

    std::cout << "Mean BGR: " << mean_b << ", " << mean_g << ", " << mean_r << std::endl;

    return "";
  }

  std::vector<double> getColorOfObject() {
    if (!hasRecognition()) {
      // @todo: Manually get the color of the view.
      std::cout << "Camera doesn't have recognitionObject" << std::endl;
      //return std::vector<double>({ 0, 0, 0 });
      return std::vector<double>({ 0 });
    }
    int num_objects = getRecognitionNumberOfObjects();
    if (num_objects == 0) {
      std::cout << "Camera doesn't recognize any objects" << std::endl;
      //return std::vector<double>({ 0, 0, 0 });
      return std::vector<double>();
    }
    webots::CameraRecognitionObject recognized_obj = getRecognitionObjects()[0];
    std::cout << "RecognitionColor: " << recognized_obj.colors[0] << ", " << recognized_obj.colors[1] << ", " << recognized_obj.colors[2] << std::endl;
    //return std::vector<double>({ recognized_obj.colors[0], recognized_obj.colors[1], recognized_obj.colors[2] });
    return std::vector<double>({ (double)recognized_obj.id });
  }
};

