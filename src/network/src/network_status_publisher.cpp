#include <iostream>

#include <stdexcept>

#include <sstream>

#include <vector>

#include <string>

#include <cstdlib>

#include <cstring>

#include <regex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include <chrono>

#include <memory>

class NetworkStatusPublisher: public rclcpp::Node {
    public: rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr publisher;
    NetworkStatusPublisher(): Node("network_status_publisher"),
    counter(0) {
        publisher = this -> create_publisher < std_msgs::msg::String > ("robot/network_status", 10);
        timer = this -> create_wall_timer(std::chrono::seconds(1), std::bind( & NetworkStatusPublisher::publish_network_status, this));
    }
    private: std::string call_out_cmd(const std::string & command_string) {
        std::string output;
        FILE * pipe = popen(command_string.c_str(), "r");
        if(!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        char buffer[128];
        while(fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            output += buffer;
        }
        int pclose_rc = pclose(pipe);
        if(pclose_rc == -1) {
            throw std::runtime_error("pclose() failed!");
        }
        int exit_code = WEXITSTATUS(pclose_rc);
        if(exit_code != 0) {
            throw std::runtime_error("Command execution failed!");
        }
        return output;
    }
    std::string get_ssid_from_iw_dev_output() {
        try {
            std::string output = call_out_cmd("iw dev");
            std::regex ssid_pattern("\\bssid\\s(.+)\\b");
            std::smatch ssid_match;
            if(std::regex_search(output, ssid_match, ssid_pattern)) {
                return ssid_match.str(1);
            } else {
                return "";
            }
        } catch (const std::exception & e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return "";
        }
    }
    std::string which_type_on() {
        try {
            FILE * pipe = popen("sudo nmcli connection show --active", "r");
            if(!pipe) {
                throw std::runtime_error("popen() failed!");
            }
            std::vector < std::string > lines;
            char buffer[128];
            while(fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                lines.push_back(buffer);
            }
            int pclose_rc = pclose(pipe);
            if(pclose_rc == -1) {
                throw std::runtime_error("pclose() failed!");
            }
            int exit_code = WEXITSTATUS(pclose_rc);
            if(exit_code != 0) {
                throw std::runtime_error("Command execution failed!");
            }
            if(lines.empty()) {
                return "no_connection";
            }
            std::string first_line = lines[1];
            if(first_line.find("Hotspot") != std::string::npos) {
                return "hotspot";
            } else if(first_line.find(get_ssid_from_iw_dev_output()) != std::string::npos) {
                return "wifi";
            } else {
                return "no_connection";
            }
        } catch (const std::exception & e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return "no_connection";
        }
    }
    std::string give_ip() {
        std::string ip = call_out_cmd("robonet-getip");
        return ip;
    }
    std::string center_ip_address(const std::string & ip_address, int total_width = 15) {
        int padding = (total_width - ip_address.length()) / 2;
        std::string formatted_string = std::string(padding, ' ') + ip_address + std::string(padding, ' ');
        if((total_width - ip_address.length()) % 2 != 0) {
            formatted_string += ' ';
        }
        return formatted_string;
    }
    public: int counter;
    std::string build_json() {
        int mode_int;
        std::string status = which_type_on();
        std::string info = get_ssid_from_iw_dev_output();
        std::string ip = center_ip_address(give_ip());
        if(status == "wifi") {
            mode_int = 1;
        } else if(status == "hotspot") {
            mode_int = 2;
        } else if(status == "no_connection") {
            mode_int = 0;
        } else {
            mode_int = 0;
        }
        std::string mode = std::to_string(mode_int);
        std::string json_string = "{";
        json_string += "\"mode\": \"" + mode + "\",";
        json_string += " \"status\": \"" + status + "\",";
        json_string += "\"info\": \"" + info + "\",";
        json_string += "\"ip\": \"" + ip + "\"";
        json_string += "}";
        return json_string;
    }
    void publish_network_status() {
        auto message = std_msgs::msg::String();
        message.data = build_json();
        publisher -> publish(message);
    }
};
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared < NetworkStatusPublisher > ();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
