#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    mSubscription = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
      "cipher_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    mClient = this->create_client<cipher_interfaces::srv::CipherAnswer>("cipher_check");
  }

private:
  void topic_callback(const cipher_interfaces::msg::CipherMessage::SharedPtr encryptedMessage) 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", encryptedMessage->message.c_str());
    mDecryptedMessage = decryptTheMessage(encryptedMessage->message, encryptedMessage->key);
    RCLCPP_INFO(this->get_logger(), "I decryted: '%s'", mDecryptedMessage.c_str());

    if (mClient->service_is_ready()) {
    auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
    request->answer = mDecryptedMessage;

    mClient->async_send_request(request, 
      std::bind(&MinimalSubscriber::handle_response, this, std::placeholders::_1));

  } else {
    RCLCPP_WARN(this->get_logger(), "Server not ready, guess not sent.");
  }
  }
  
  void handle_response(rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedFuture future)
  {
    auto response = future.get();
    if (response->result) {
        RCLCPP_INFO(this->get_logger(), "answer was correct!");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(this->get_logger(), "answer was wrong.");
    }
  }

  std::string decryptTheMessage(std::string input, int shift) {
    std::string result = input; 
    for (char &c : result) {
        if (isupper(c)) {
            c = (static_cast<int>(c) - shift - 'A' + 26) % 26 + 'A';
        }
        else if (islower(c)) {
            c = (static_cast<int>(c) - shift - 'a' + 26) % 26 + 'a';
        }
    }
    return result;
  }
  std::string mDecryptedMessage {};
  rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr mSubscription;
  rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr mClient;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
