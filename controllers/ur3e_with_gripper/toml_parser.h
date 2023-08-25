#pragma once
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include "toml.h"
#include "AERA_Protobuf/tcp_data_message.pb.h"

namespace toml_parser {

  struct prop {
    std::string name;
    tcp_io_device::VariableDescription_DataType data_type;
    std::vector<uint64_t> dimensions;
    std::string opcode_handle;


    friend std::ostream& operator<<(std::ostream& os, const prop p) {
      os << "Name: " << p.name << std::endl;
      os << "Type: " << p.data_type << std::endl;
      os << "Dimensions: [ ";
      for (auto it = p.dimensions.begin(); it != p.dimensions.end(); ++it) {
        if (it != p.dimensions.begin()) {
          os << ", ";
        }
        os << *it;
      }
      os << " ]" << std::endl;
      os << "OpCode Handle: " << p.opcode_handle << std::endl;
      return os;
    }
  };
  struct cmd {
    std::string name;
    tcp_io_device::VariableDescription_DataType data_type;
    std::vector<uint64_t> dimensions;
    std::string opcode_handle;


    friend std::ostream& operator<<(std::ostream& os, const cmd c) {
      os << "Name: " << c.name << std::endl;
      os << "Type: " << c.data_type << std::endl;
      os << "Dimensions: [ ";
      for (auto it = c.dimensions.begin(); it != c.dimensions.end(); ++it) {
        if (it != c.dimensions.begin()) {
          os << ", ";
        }
        os << *it;
      }
      os << " ]" << std::endl;
      os << "OpCode Handle: " << c.opcode_handle << std::endl;
      return os;
    }
  };
  struct entity {
    std::string name;
    std::vector<prop> properties;
    std::vector<cmd> commands;


    friend std::ostream& operator<<(std::ostream& os, const entity e) {
      os << "------------" << std::endl;
      os << "Name: " << e.name << std::endl;
      if (e.properties.size() != 0) {
        os << "------" << std::endl;
        os << "Properties: " << std::endl;
        for (auto it = e.properties.begin(); it != e.properties.end(); ++it) {
          os << "---" << std::endl;
          os << *it << std::endl;
        }
      }
      if (e.commands.size() != 0) {
        os << "------" << std::endl;
        os << "Commands: " << std::endl;
        for (auto it = e.commands.begin(); it != e.commands.end(); ++it) {
          os << "---" << std::endl;
          os << *it << std::endl;
        }
      }
      return os;
    }
  };

  class TOMLParser
  {
  private:
    std::map<std::string, entity> entities_;
    std::map<std::string, prop> properties_;
    std::map<std::string, cmd> commands_;
  public:
    TOMLParser() {};
    ~TOMLParser() {};

    void parse(std::string file_name) {
      std::ifstream ifs(file_name);
      toml::ParseResult pr = toml::parse(ifs);

      if (!pr.valid()) {
        std::cout << pr.errorReason << std::endl;
        return;
      }

      const toml::Value& v = pr.value;

      const toml::Value* c = v.find("commands");
      if (!(c && c->is<toml::Array>())) {
        std::cout << "ERROR when loading " << file_name << ": commands is not an array" << std::endl;
      }

      const toml::Value* p = v.find("properties");
      if (!(p && p->is<toml::Array>())) {
        std::cout << "ERROR when loading " << file_name << ": properties is not an array" << std::endl;
      }

      const toml::Value* e = v.find("entities");
      if (!(e && e->is<toml::Array>())) {
        std::cout << "ERROR when loading " << file_name << ": entities is not an array" << std::endl;
      }


      toml::Array c_list = c->as<toml::Array>();
      for (auto it = c_list.begin(); it != c_list.end(); ++it) {
        auto c_struct = buildCommand(*it);
        commands_[c_struct.name] = c_struct;
      }

      toml::Array p_list = p->as<toml::Array>();
      for (auto it = p_list.begin(); it != p_list.end(); ++it) {
        auto p_struct = buildProperty(*it);
        properties_[p_struct.name] = p_struct;
      }

      toml::Array e_list = e->as<toml::Array>();
      for (auto it = e_list.begin(); it != e_list.end(); ++it) {
        entity e_struct;
        e_struct.name = it->find("name")->as<std::string>();

        auto val = it->find("properties");
        if (val && val->is<toml::Array>()) {
          auto props = val->as<toml::Array>();
          for (auto p = props.begin(); p != props.end(); ++p) {
            e_struct.properties.push_back(properties_[p->as<std::string>()]);
          }
        }

        auto c_val = it->find("commands");
        if (c_val && c_val->is<toml::Array>()) {
          auto cmds = c_val->as<toml::Array>();
          for (auto c = cmds.begin(); c != cmds.end(); ++c) {
            e_struct.commands.push_back(commands_[c->as<std::string>()]);
          }
        }
        entities_[e_struct.name] = e_struct;
      }

    }

    cmd buildCommand(const toml::Value& v) {
      cmd c;

      auto val = v.find("name");
      if (val && val->is<std::string>()) {
        c.name = val->as<std::string>();
      }
      else {
        std::cout << "ERROR: Name of command not found. This will lead to errors." << std::endl;
      }

      val = v.find("type");
      if (val && val->is<std::string>()) {
        c.data_type = getDataType(val->as<std::string>());
      }
      else {
        std::cout << "ERROR: DataType of command not found. This will lead to errors." << std::endl;
      }

      val = v.find("dimensionality");
      if (val && val->is<toml::Array>()) {
        toml::Array dim = val->as<toml::Array>();
        for (auto it = dim.begin(); it != dim.end(); ++it) {
          c.dimensions.push_back(it->as<int64_t>());
        }
      }
      else {
        std::cout << "ERROR: Dimensionality of command not found. This will lead to errors." << std::endl;
      }
      val = v.find("opcode_handle");
      if (val && val->is<std::string>()) {
        c.opcode_handle = val->as<std::string>();
      }
      return c;
    }

    prop buildProperty(const toml::Value& v) {
      prop p;

      auto val = v.find("name");
      if (val && val->is<std::string>()) {
        p.name = val->as<std::string>();
      }
      else {
        std::cout << "ERROR: Name of property not found. This will lead to errors." << std::endl;
      }

      val = v.find("type");
      if (val && val->is<std::string>()) {
        p.data_type = getDataType(val->as<std::string>());
      }
      else {
        std::cout << "ERROR: DataType of property not found. This will lead to errors." << std::endl;
      }

      val = v.find("dimensionality");
      if (val && val->is<toml::Array>()) {
        toml::Array dim = val->as<toml::Array>();
        for (auto it = dim.begin(); it != dim.end(); ++it) {
          p.dimensions.push_back(it->as<int64_t>());
        }
      }
      else {
        std::cout << "ERROR: Dimensionality of property not found. This will lead to errors." << std::endl;
      }
      val = v.find("opcode_handle");
      if (val && val->is<std::string>()) {
        p.opcode_handle = val->as<std::string>();
      }
      else {
        p.opcode_handle = "";
      }
      return p;
    }

    static tcp_io_device::VariableDescription_DataType getDataType(std::string t) {
      if (t == "double") {
        return tcp_io_device::VariableDescription_DataType_DOUBLE;
      }
      else if (t == "communication_id") {
        return tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID;
      }
      else if (t == "int") {
        return tcp_io_device::VariableDescription_DataType_INT64;
      }
      else if (t == "bool") {
        return tcp_io_device::VariableDescription_DataType_BOOL;
      }
      else if (t == "string") {
        return tcp_io_device::VariableDescription_DataType_STRING;
      }
      else if (t == "bytes") {
        return tcp_io_device::VariableDescription_DataType_BYTES;
      }
      else {
        std::cout << "ERROR: data type " << t << "in TOML file unknown! Returning BYTES as default." << std::endl;
        return tcp_io_device::VariableDescription_DataType_BYTES;
      }
    }

    static std::string getDataTypeName(tcp_io_device::VariableDescription_DataType t) {
      if (t == tcp_io_device::VariableDescription_DataType_DOUBLE) {
        return "double";
      }
      else if (t == tcp_io_device::VariableDescription_DataType_COMMUNICATION_ID) {
        return "communication_id";
      }
      else if (t == tcp_io_device::VariableDescription_DataType_INT64) {
        return "int";
      }
      else if (t == tcp_io_device::VariableDescription_DataType_BOOL) {
        return "bool";
      }
      else if (t == tcp_io_device::VariableDescription_DataType_STRING) {
        return "string";
      }
      else if (t == tcp_io_device::VariableDescription_DataType_BYTES) {
        return "bytes";
      }
      else {
        return "DataTypeNotFound";
      }
    }

    std::map<std::string, entity> entities() { return entities_; }
    std::map<std::string, prop> properties() { return properties_; }
    std::map<std::string, cmd> commands() { return commands_; }

    std::vector<std::string> entityNames() {
      std::vector<std::string> out;
      for (auto it = entities_.begin(); it != entities_.end(); ++it) {
        out.push_back(it->first);
      }
      return out;
    }
    std::vector<std::string> propertyNames() {
      std::vector<std::string> out;
      for (auto it = properties_.begin(); it != properties_.end(); ++it) {
        out.push_back(it->first);
      }
      return out;
    }
    std::vector<std::string> commandNames() {
      std::vector<std::string> out;
      for (auto it = commands_.begin(); it != commands_.end(); ++it) {
        out.push_back(it->first);
      }
      return out;
    }
  };





} // namespace toml_parser



