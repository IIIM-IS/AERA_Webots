#pragma once

#include <chrono>
#include <algorithm>
#include <vector>
#include <memory>
#include "../../submodules/AERA_Protobuf/tcp_connection.h"
#include "../../submodules/AERA_Protobuf/utils.h"
#include "../../submodules/AERA_Protobuf/tcp_data_message.pb.h"
#include <webots/Supervisor.hpp>

class AERAController
{
public:

  AERAController();
  ~AERAController();

  virtual int start() = 0;

  int startConnection();

  void stop();

  webots::Supervisor* robot_;
protected:

  int robot_time_step_;

  int id_counter_;
  std::map<int, std::string> id_string_mapping_;
  std::map<std::string, int> string_id_mapping_;
  std::vector<tcp_io_device::MetaData> objects_meta_data_;
  std::vector<tcp_io_device::MetaData> commands_meta_data_;

  std::shared_ptr<tcp_io_device::SafeQueue> receive_queue_;
  std::shared_ptr<tcp_io_device::SafeQueue> send_queue_;
  std::shared_ptr<tcp_io_device::TCPConnection> tcp_connection_;

  bool aera_started_ = false;
  bool diagnostic_mode_ = false;

  void fillIdStringMaps(std::vector<std::string> names);

  void sendSetupMessage(std::vector<tcp_io_device::MetaData> objects, std::vector<tcp_io_device::MetaData> commands);

  void waitForStartMsg(int timeout = 0);

  void handleStartMsg(std::unique_ptr<tcp_io_device::TCPMessage> msg);

  virtual void run() = 0;

  static std::vector<tcp_io_device::MsgData> dataMsgToMsgData(std::unique_ptr<tcp_io_device::TCPMessage> msg) {
    std::vector<tcp_io_device::MsgData> out_vec;
    auto variables = msg->datamessage().variables();
    for (auto var = variables.begin(); var != variables.end(); ++var) {
      out_vec.push_back(tcp_io_device::MsgData(&(*var)));
    }
    return out_vec;
  }

  template<typename T>

  tcp_io_device::MsgData createMsgData(tcp_io_device::MetaData meta_data, std::vector<T> data);
  tcp_io_device::MsgData createMsgData(tcp_io_device::MetaData meta_data, std::string data);

  void sendDataMessage(std::vector<tcp_io_device::MsgData> msg_data);

  virtual void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) = 0;
};

