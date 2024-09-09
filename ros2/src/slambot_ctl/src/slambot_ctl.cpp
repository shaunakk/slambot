#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include "geometry_msgs/msg/twist.hpp"

// #include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;
using namespace std::chrono_literals;
#define head1 0xAA
#define head2 0x55
#define sendType_velocity 0x11
#define sendType_pid 0x12
#define sendType_params 0x13
rclcpp::Time cmd_time;

double x = 0.0;
double y = 0.0;
double yaw = 0.0;
int baud_rate = 115200;
io_service iosev;
serial_port sp(iosev); // Define the serial port for transmission

std::string port_name = std::string("/dev/ttyACM0");
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
uint8_t checksum(uint8_t *buf, size_t len)
{
  uint8_t sum = 0x00;
  for (int i = 0; i < len; i++)
  {
    sum += *(buf + i);
  }

  return sum;
}
void SetVelocity(double x, double y, double yaw)
{
  static uint8_t tmp[11];
  tmp[0] = head1;
  tmp[1] = head2;
  tmp[2] = 0x0b;
  tmp[3] = sendType_velocity;
  tmp[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
  tmp[5] = ((int16_t)(x * 1000)) & 0xff;
  tmp[6] = ((int16_t)(y * 1000) >> 8) & 0xff;
  tmp[7] = ((int16_t)(y * 1000)) & 0xff;
  tmp[8] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
  tmp[9] = ((int16_t)(yaw * 1000)) & 0xff;
  tmp[10] = checksum(tmp, 10);
  write(sp, buffer(tmp, 11));
}

// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0)
//     {
//       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//       timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

//   private:
//     void timer_callback()
//     {
//       auto message = std_msgs::msg::String();
//       message.data = "Hello, world! " + std::to_string(count_++);
//       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//       publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };

void topic_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{

  x = msg->linear.x;
  y = msg->linear.y;
  yaw = msg->angular.z;
  cmd_time = rclcpp::Clock{}.now();

  printf("X: %d\n", x);
  printf("Y: %d\n", y);
  printf("YAW: %d\n", yaw);

}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // rclcpp::shutdown();
  // ros::NodeHandle n;
  // ros::NodeHandle pn("~");
  boost::system::error_code ec;
  sp.open(port_name, ec);
  sp.set_option(serial_port::baud_rate(baud_rate));
  sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp.set_option(serial_port::parity(serial_port::parity::none));
  sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp.set_option(serial_port::character_size(8));

  //  //Create serial port receiving task
  // thread serial_thread(boost::bind(serial_task));

  rclcpp::Time current_time;
  rclcpp::Time last_time;

  current_time = rclcpp::Clock{}.now();
  last_time = rclcpp::Clock{}.now();
  auto node = rclcpp::Node::make_shared("controller");
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, topic_callback);
  while (rclcpp::ok())
  {
    current_time = rclcpp::Clock{}.now();
    if ((current_time - last_time).seconds() > 0.02)
    {
      last_time = current_time;

      if ((current_time - cmd_time).seconds() >1)
      {
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
      }
      SetVelocity(x, y, yaw);
    }
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}