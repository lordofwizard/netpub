#include <chrono>
#include <memory>
#include <string>
#include <array>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <regex>

class NetworkStatusPublisher : public rclcpp::Node {
public:
    NetworkStatusPublisher()
    : Node("network_status_publisher"), counter(0) {
        json_data_publisher = this->create_publisher<std_msgs::msg::String>("robot/network_status", 10);
        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NetworkStatusPublisher::publish_network_status, this));
        some_ip_address = "  Fetching IP  ";
    }

private:
    void publish_network_status() {
        auto connection = connection_type();
        std::string status, info, ip;
        int network_status = 69;


        RCLCPP_INFO(this->get_logger(), "Connection Type: %s", connection.c_str());

        if (connection != "nocon") {
            // Print the output of the robonet-getip command for debugging
            auto ip_output = execute_command("robonet-getip");
            RCLCPP_INFO(this->get_logger(), "Output of robonet-getip: %s", ip_output.c_str());
            
            some_ip_address = ip_output;
            if (some_ip_address.find("None") != std::string::npos) {
                some_ip_address = "  Fetching IP  ";
            }
        }
        try {
            if (connection == "wifi") {
                network_status = 1;
                status = connection;
                auto temp_ssid = grep_ssid();
                if (!temp_ssid.empty()) {
                    info = center_ip_address(temp_ssid);
                } else {
                    info = center_ip_address("Lost Connection");
                }
                ip = some_ip_address;
            } else if (connection == "hotspot") {
                network_status = 2;
                status = connection;
                info = center_ip_address(execute_command("echo $USER"));
                ip = some_ip_address;
            } else if (connection == "dual" || connection == "dualw") {
                network_status = (connection == "dual") ? 3 : 4;
                status = "wifi";
                info = center_ip_address(grep_ssid());
                ip = some_ip_address;
            } else if (connection == "nocon") {
                network_status = 0;
                status = "nocon";
                info = center_ip_address(" No Connection ");
                ip = center_ip_address("    Trying    ");
            }
        } catch (const std::exception& e) {
            network_status = 4;
        }

        ip.erase(std::remove(ip.begin(), ip.end(), '\n'), ip.end());

        std::cout << "Mode: " << network_status << std::endl;
        std::cout << "Status: " << status << std::endl;
        std::cout << "Info: " << info << std::endl;
        std::cout << "IP: " << ip << std::endl;

        auto json_obj = R"({
            "mode" : )" + std::to_string(network_status) + R"(,
            "status": ")" + status + R"(",
            "info" : ")" + info + R"(",
            "ip"   : ")" + ip + R"("
        })";
        auto msg = std_msgs::msg::String();
        msg.data = json_obj;
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
        RCLCPP_INFO(this->get_logger(), "%d", counter);
        json_data_publisher->publish(msg);
        counter++;
    }

    std::string connection_type() {
        auto active_connection = execute_command("sudo nmcli connection show --active");
        if (active_connection.find("wifi") != std::string::npos) {
            if (std::count(active_connection.begin(), active_connection.end(), '\n') == 3) {
                return (active_connection.find("Hotspot") != std::string::npos) ? "dual" : "dualw";
            } else {
                return (active_connection.find("Hotspot") != std::string::npos) ? "hotspot" : "wifi";
            }
        } else {
            return "nocon";
        }
    }

        std::string execute_command(const std::string& command) {
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        std::string result;
        char buffer[128];
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL) {
                result += buffer;
                // Print the command output
                std::cout << "Command Output: " << buffer;
            }
        }
        pclose(pipe);
        return result;
    }

    std::string grep_ssid() {
        auto output = execute_command("iwgetid");
        std::smatch match;
        std::regex pattern("ESSID:\"([^\"]*)\"");
        if (std::regex_search(output, match, pattern)) {
            return match.str(1);
        }
        return "";
    }

    std::string center_ip_address(const std::string& ip_address, int total_width = 15) {
        auto padding = (total_width - ip_address.length()) / 2;
        return std::string(padding, ' ') + ip_address + std::string(padding, ' ');
    }

    rclcpp::TimerBase::SharedPtr timer;
    int counter;
    std::string some_ip_address;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared
    <NetworkStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

