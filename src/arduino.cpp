/*
  ____                                   _             _
 / ___|  ___  _   _ _ __ ___ ___   _ __ | |_   _  __ _(_)_ __
 \___ \ / _ \| | | | '__/ __/ _ \ | '_ \| | | | |/ _` | | '_ \
  ___) | (_) | |_| | | | (_|  __/ | |_) | | |_| | (_| | | | | |
 |____/ \___/ \__,_|_|  \___\___| | .__/|_|\__,_|\__, |_|_| |_|
                                  |_|            |___/
# A Template for ArduinoPlugin, a Source Plugin
# Generated by the command: plugin -t source -d arduino_plugin arduino
# Hostname: Fram-IV.local
# Current working directory: /Users/p4010/Develop/MADS_plugins
# Creation date: 2024-07-31T08:52:13.115+0200
# NOTICE: MADS Version 1.0.0
*/
// Mandatory included headers
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>
#include <source.hpp>
// other includes as needed here
#include <thread>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

// Define the name of the plugin
#ifndef PLUGIN_NAME
#define PLUGIN_NAME "arduino"
#endif

// Load the namespaces
using namespace std;
using json = nlohmann::json;

// Plugin class. This shall be the only part that needs to be modified,
// implementing the actual functionality
class ArduinoPlugin : public Source<json> {

  return_type setup() {
    if (_serialPort == nullptr) {
#ifndef _WIN32
      if (filesystem::exists(_params["port"].get<string>()) == false) {
        if (!_params["silent"]) {
          cerr << "Error: port " << _params["port"].get<string>()
               << " does not exist" << endl;
        }
        _error = "Port does not exist";
        return return_type::critical;
      }
#endif
      try {
        _serialPort = new serial::Serial(
            _params["port"].get<string>().c_str(),
            _params["baudrate"].get<unsigned>(),
            serial::Timeout::simpleTimeout(
                _params["connection_timeout"].get<unsigned>()));
      } catch (std::exception &e) {
        if (!_params["silent"])
          cerr << "Error: " << e.what() << endl;
        _error = e.what();
        return return_type::critical;
      }
      if (!_serialPort->isOpen()) {
        if (!_params["silent"])
          cerr << "Error: could not open port " << _params["port"] << endl;
        _error = "Could not open port";
        return return_type::critical;
      } else {
        if (!_params["silent"])
          cerr << "Connection with " << _params["port"] << " opened" << endl;
      }
    }
    return return_type::success;
  }

public:
  ~ArduinoPlugin() {
    if (_serialPort != nullptr) {
      if (_serialPort->isOpen())
        _serialPort->close();
      delete _serialPort;
      if (!_params["silent"])
        cerr << "Connection with " << _params["port"] << " closed" << endl;
    }
  }

  string kind() override { return PLUGIN_NAME; }

  return_type get_output(json &out,
                         std::vector<unsigned char> *blob = nullptr) override {
    string line;
    bool success = false;
    out.clear();
    if (!_serialPort->isOpen()) {
      _serialPort->open();
    }
    do {
      line.clear();
      line = _serialPort->readline();
      try {
        out = json::parse(line);
        success = true;
      } catch (json::exception &e) {
      }
    } while (success == false);
    if (!_agent_id.empty())
      out["agent_id"] = _agent_id;
    return return_type::success;
  }

  void set_params(void const *params) override {
    Source::set_params(params);
    _params["port"] = "/dev/ttyUSB0";
    _params["baudrate"] = 115200;
    _params["silent"] = true;
    _params["connection_timeout"] = 5000u;
    _params["cfg_cmd"] = "";
    _params.merge_patch(*(json *)params);
    if (setup() != return_type::success) {
      throw std::runtime_error("Error setting up serial port");
    }
    if (_params.find("cfg_cmd") != _params.end() && !_params["cfg_cmd"].empty()) {
      _serialPort->write(_params["cfg_cmd"].get<string>().c_str());
      _serialPort->write("\n");
    }
  }

  map<string, string> info() override {
    return {{"port", _params["port"].get<string>()},
            {"baudrate", to_string(_params["baudrate"].get<unsigned>())},
            {"cfg_cmd", _params["cfg_cmd"].get<string>()},
            {"connection_timeout",
             to_string(_params["connection_timeout"].get<unsigned>())}};
  };

private:
  json _data, _params;
  serial::Serial *_serialPort = nullptr;
};

/*
  ____  _             _             _      _
 |  _ \| |_   _  __ _(_)_ __     __| |_ __(_)_   _____ _ __
 | |_) | | | | |/ _` | | '_ \   / _` | '__| \ \ / / _ \ '__|
 |  __/| | |_| | (_| | | | | | | (_| | |  | |\ V /  __/ |
 |_|   |_|\__,_|\__, |_|_| |_|  \__,_|_|  |_| \_/ \___|_|
                |___/
Enable the class as plugin
*/
INSTALL_SOURCE_DRIVER(ArduinoPlugin, json)

/*
                  _
  _ __ ___   __ _(_)_ __
 | '_ ` _ \ / _` | | '_ \
 | | | | | | (_| | | | | |
 |_| |_| |_|\__,_|_|_| |_|

For testing purposes, when directly executing the plugin
*/

#include <csignal>

void enumerate_ports() {
  vector<serial::PortInfo> devices_found = serial::list_ports();
  vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("%s: %s, %s\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

static bool running = true;

int main(int argc, char const *argv[]) {
  ArduinoPlugin plugin;
  json output;
  unsigned long i = 0;

  if (argc < 2) {
    cout << "Usage: " << argv[0] << " <port> | -e" << endl;
    return 1;
  }

  if (string(argv[1]) == "-e") {
    enumerate_ports();
    return 0;
  }

  signal(SIGINT, [](int) { running = false; });

  // Set parameters
  json params;
  params["port"] = argv[1];
  params["baudrate"] = 115200;
  params["silent"] = false;
  params["cfg_cmd"] = "500p";
  plugin.set_params(&params);

  while (running) {
    plugin.get_output(output);
    cout << "message #" << setfill('_') << setw(6) << i++ << ": " << output
         << endl;
  }

  return 0;
}
