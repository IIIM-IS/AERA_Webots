#include "aera_controller.h"

AERAController::AERAController()
{
  id_counter_ = 0;
  id_string_mapping_ = std::map<int, std::string>();
  string_id_mapping_ = std::map<std::string, int>();

  receive_queue_ = std::make_shared<tcp_io_device::SafeQueue>(100);
  send_queue_ = std::make_shared<tcp_io_device::SafeQueue>(1);
  tcp_connection_ = std::make_shared<tcp_io_device::TCPConnection>(receive_queue_, send_queue_, 8);
}

AERAController::~AERAController()
{
  tcp_connection_->stop();
}

int AERAController::startConnection()
{
  std::cout << "Establishing TCP connection" << std::endl;
  int err = tcp_connection_->establishConnection("127.0.0.1", "8080");
  if (err != 0) {
    std::cout << "ERROR: Could not establish a connection. Shutting down..." << std::endl;
    return -1;
  }

  tcp_connection_->start();

  return 0;

}

void AERAController::stop()
{
  tcp_connection_->stop();
}

void AERAController::fillIdStringMaps(std::vector<std::string> names) {
  for (auto it = names.begin(); it != names.end(); ++it) {
    id_string_mapping_[id_counter_] = *it;
    string_id_mapping_[*it] = id_counter_;
    id_counter_++;
  }
}


void AERAController::sendSetupMessage(std::vector<tcp_io_device::MetaData> objects, std::vector<tcp_io_device::MetaData> commands)
{
  // Create a new setup message
  std::unique_ptr<tcp_io_device::TCPMessage> msg = std::make_unique<tcp_io_device::TCPMessage>();
  msg->set_messagetype(tcp_io_device::TCPMessage::SETUP);
  tcp_io_device::SetupMessage* setup_msg = msg->mutable_setupmessage();
  auto entities_msg = setup_msg->mutable_entities();
  auto objects_msg = setup_msg->mutable_objects();
  auto cmd_msg = setup_msg->mutable_commands();
  auto cmd_descriptions_msg = setup_msg->mutable_commanddescriptions();

  std::vector<int> sent_entities;
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    objects_msg->operator[](id_string_mapping_[it->getID()]) = it->getID();
    if (std::find(sent_entities.begin(), sent_entities.end(), it->getEntityID()) == sent_entities.end()) {
      sent_entities.push_back(it->getEntityID());
      entities_msg->operator[](id_string_mapping_[it->getEntityID()]) = it->getEntityID();
    }
  }

  std::vector<tcp_io_device::CommandDescription> cmd_descriptions;
  for (auto it = commands.begin(); it != commands.end(); ++it) {
    cmd_msg->operator[](id_string_mapping_[it->getID()]) = it->getID();
    tcp_io_device::CommandDescription cmd_description;
    cmd_description.set_name(id_string_mapping_[it->getID()]);
    it->toMutableVariableDescription(cmd_description.mutable_description());
    cmd_descriptions.push_back(cmd_description);
  }
  cmd_descriptions_msg->Add(cmd_descriptions.begin(), cmd_descriptions.end());

  send_queue_->enqueue(std::move(msg));
}

void AERAController::waitForStartMsg(int timeout) {
  uint64_t start = std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::system_clock::now().time_since_epoch()).count();
  while (true)
  {
    auto msg = receive_queue_->dequeue();
    if (!msg) {
      continue;
    }
    if (msg->messagetype() == tcp_io_device::TCPMessage_Type_START) {
      std::cout << "Received START message" << std::endl;
      handleStartMsg(std::move(msg));
      break;
    }
    if (timeout != 0) {
      uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();
      if (current_time - start >= timeout) {
        std::cout << "Timeout occured after " << current_time - start << " milliseconds while waiting for START message" << std::endl;
        break;
      }
    }
  }
}

void AERAController::handleStartMsg(std::unique_ptr<tcp_io_device::TCPMessage> msg) {
  aera_started_ = true;
  diagnostic_mode_ = msg->startmessage().diagnosticmode();
}

template<typename T>
tcp_io_device::MsgData AERAController::createMsgData(tcp_io_device::MetaData meta_data, std::vector<T> data) {
  return tcp_io_device::MsgData::createNewMsgData(meta_data, data);
}

void AERAController::sendDataMessage(std::vector<tcp_io_device::MsgData> msg_data) {
  std::unique_ptr<tcp_io_device::TCPMessage> msg = std::make_unique<tcp_io_device::TCPMessage>();
  msg->set_messagetype(tcp_io_device::TCPMessage::DATA);
  tcp_io_device::DataMessage* data_msg = msg->mutable_datamessage();
  for (auto it = msg_data.begin(); it != msg_data.end(); ++it) {
    tcp_io_device::ProtoVariable* var = data_msg->add_variables();
    it->toMutableProtoVariable(var);
  }
  send_queue_->enqueue(std::move(msg));
}




template tcp_io_device::MsgData AERAController::createMsgData<int>(tcp_io_device::MetaData meta_data, std::vector<int> data);
template tcp_io_device::MsgData AERAController::createMsgData<double>(tcp_io_device::MetaData meta_data, std::vector<double> data);
template tcp_io_device::MsgData AERAController::createMsgData<std::string>(tcp_io_device::MetaData meta_data, std::vector<std::string> data);
