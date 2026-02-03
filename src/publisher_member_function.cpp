#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    this->declare_parameter<std::string>("message", "");
    this->declare_parameter<int>("key", 0);

    mMessage = this->get_parameter("message").as_string();
    mKey = this->get_parameter("key").as_int();

    mPublisher = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_topic", 10);  
    
    mService = this->create_service<cipher_interfaces::srv::CipherAnswer>(
        "cipher_check", std::bind(&MinimalPublisher::check, this, std::placeholders::_1, std::placeholders::_2));
    
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  void check(std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
          std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response>      response)
  {

    if (request->answer == mMessage) {
        response->result = true;
        RCLCPP_INFO(this->get_logger(), "Checked. Sending back response: true"); 
        rclcpp::shutdown();
    }
    else {
        response->result = false;
        RCLCPP_INFO(this->get_logger(), "Checked. Sending back response: false"); 
    }
  }

private:
  std::string mMessage {};  
  int mKey {};
  bool mMessageSent {false};
  void timer_callback()
  {
    if (mPublisher->get_subscription_count() > 0 && mMessageSent == false) {
        auto msg_to_publish = cipher_interfaces::msg::CipherMessage();
        msg_to_publish.message = encryptTheMessage(mMessage, mKey);
        msg_to_publish.key = mKey;

        msg_to_publish.header.stamp = this->get_clock()->now();
        msg_to_publish.header.frame_id = "minimal_publisher";

        RCLCPP_INFO(this->get_logger(), "Given: '%s'", mMessage.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_to_publish.message.c_str());      // publishes to terminal but not needed for functionality. 
        mPublisher->publish(msg_to_publish);
        mMessageSent = true;
    } else if (mMessageSent == false ) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for a participant to join...");
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr mPublisher;
  rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr mService;
  size_t count_;

  std::string encryptTheMessage(std::string input, int shift) {
    std::string result = input; 
    for (char &c : result) {
        if (isupper(c)) {
            c = (static_cast<int>(c) + shift - 'A') % 26 + 'A';
        }
        else if (islower(c)) {
            c = (static_cast<int>(c) + shift - 'a') % 26 + 'a';
        }
    }
    return result;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
