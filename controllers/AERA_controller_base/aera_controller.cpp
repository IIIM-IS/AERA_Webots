#include "aera_controller.h"

AERAController::AERAController()
{
  id_counter_ = 0;
  id_string_mapping_ = std::map<int, std::string>();
  string_id_mapping_ = std::map<std::string, int>();

  receive_queue_ = std::make_shared<tcp_io_device::SafeQueue>(1);
  send_queue_ = std::make_shared<tcp_io_device::SafeQueue>(1);
  tcp_connection_ = std::make_shared<tcp_io_device::TCPConnection>(receive_queue_, send_queue_, 8);

  reconnection_type_ = tcp_io_device::StartMessage_ReconnectionType_RE_INIT;
}

AERAController::~AERAController()
{
  tcp_connection_->stop();
}

std::map<std::string, std::map<std::string, tcp_io_device::MetaData>> AERAController::setup(std::string settings_file_name)
{
  toml_parser::TOMLParser parser;
  parser.parse(settings_file_name);

  std::vector<tcp_io_device::MetaData> objects;
  std::map<std::string, std::map<std::string, tcp_io_device::MetaData> > objects_map;
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
    std::cout << "Entity: " << e_it->first << " ID: " << string_id_mapping_[e_it->first] << std::endl;
    for (auto p_it = e.properties.begin(); p_it != e.properties.end(); ++p_it) {
      objects.push_back(tcp_io_device::MetaData(string_id_mapping_[e_it->first], string_id_mapping_[p_it->name], p_it->data_type, p_it->dimensions, p_it->opcode_handle));
      objects_map[e_it->first].insert(std::make_pair(p_it->name, objects.back()));
      std::cout << "Entity: " << e_it->first << " Property: " << p_it->name << " ID: " << string_id_mapping_[p_it->name] << std::endl;
    }
    for (auto c_it = e.commands.begin(); c_it != e.commands.end(); ++c_it) {
      commands.push_back(tcp_io_device::MetaData(string_id_mapping_[e_it->first], string_id_mapping_[c_it->name], c_it->data_type, c_it->dimensions, c_it->opcode_handle));
      std::cout << "Entity: " << e_it->first << " Command: " << c_it->name << " ID: " << string_id_mapping_[c_it->name] << std::endl;
    }
  }
  sendSetupMessage(objects, commands);
  return objects_map;
}

int AERAController::startConnection()
{
  std::cout << "Establishing TCP connection" << std::endl;
  int err = tcp_connection_->listenAndAwaitConnection("8080");
  // int err = tcp_connection_->establishConnection("127.0.0.1", "8080");
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
  objects_meta_data_ = objects;
  commands_meta_data_ = commands;
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

void AERAController::handleReconnect() {
  switch (reconnection_type_)
  {
  case tcp_io_device::StartMessage_ReconnectionType_RE_INIT:
    robot_->simulationReset();
    init();
    sendSetupMessage(objects_meta_data_, commands_meta_data_);
    break;
  case tcp_io_device::StartMessage_ReconnectionType_RE_SETUP:
    sendSetupMessage(objects_meta_data_, commands_meta_data_);
    break;
  default:
    break;
  }
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
    //std::cout << *it << std::endl;
    tcp_io_device::ProtoVariable* var = data_msg->add_variables();
    it->toMutableProtoVariable(var);
  }
  data_msg->set_timespan(robot_->getTime() * 1000);
  send_queue_->enqueue(std::move(msg));
}

void AERAController::sendGoalMessage(tcp_io_device::MsgData msg_data) {
  std::unique_ptr<tcp_io_device::TCPMessage> msg = std::make_unique<tcp_io_device::TCPMessage>();
  msg->set_messagetype(tcp_io_device::TCPMessage::GOAL);
  tcp_io_device::GoalMessage* goal_msg = msg->mutable_goalmessage();
  tcp_io_device::ProtoVariable* var = goal_msg->mutable_goal();
  msg_data.toMutableProtoVariable(var);
  send_queue_->enqueue(std::move(msg));
}



template tcp_io_device::MsgData AERAController::createMsgData<int64_t>(tcp_io_device::MetaData meta_data, std::vector<int64_t> data);
template tcp_io_device::MsgData AERAController::createMsgData<double>(tcp_io_device::MetaData meta_data, std::vector<double> data);
template tcp_io_device::MsgData AERAController::createMsgData<std::string>(tcp_io_device::MetaData meta_data, std::vector<std::string> data);
