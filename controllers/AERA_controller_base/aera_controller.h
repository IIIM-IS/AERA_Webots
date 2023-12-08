#pragma once

#include <chrono>
#include <algorithm>
#include <vector>
#include <memory>
#include "../../submodules/AERA_Protobuf/tcp_connection.h"
#include "../../submodules/AERA_Protobuf/utils.h"
#include "../../submodules/AERA_Protobuf/tcp_data_message.pb.h"
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "toml_parser.h"

using communication_id_t = int64_t;

class AERAController
{
public:

  AERAController();
  ~AERAController();

  virtual int start() = 0;

  virtual void init() = 0;

  std::map<std::string, std::map<std::string, tcp_io_device::MetaData> > setup(std::string settings_file_name);

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

  tcp_io_device::StartMessage_ReconnectionType reconnection_type_;

  bool aera_started_ = false;
  bool diagnostic_mode_ = false;

  void fillIdStringMaps(std::vector<std::string> names);

  void sendSetupMessage(std::vector<tcp_io_device::MetaData> objects, std::vector<tcp_io_device::MetaData> commands);

  void waitForStartMsg(int timeout = 0);

  void handleStartMsg(std::unique_ptr<tcp_io_device::TCPMessage> msg);

  virtual void run() = 0;

  virtual void handleReconnect();

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

  void sendDataMessage(std::vector<tcp_io_device::MsgData> msg_data);

  virtual void handleDataMsg(std::vector<tcp_io_device::MsgData> msg_data) = 0;
};

