#include <chrono>
#include <functional>
#include <memory>
#include <string>
 #include <stdio.h>
 #include <unistd.h>
 #include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
char key(' ');


// For non-blocking keyboard inputs
 int getch(void)
 {
   int ch;
   struct termios oldt;
   struct termios newt;
 
   // Store old settings, and copy to new settings
   tcgetattr(STDIN_FILENO, &oldt);
   newt = oldt;
 
   // Make required changes and apply the settings
   newt.c_lflag &= ~(ICANON | ECHO);
   newt.c_iflag |= IGNBRK;
   newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
   newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
   newt.c_cc[VMIN] = 1;
   newt.c_cc[VTIME] = 0;
   tcsetattr(fileno(stdin), TCSANOW, &newt);
 
   // Get the current character
   ch = getchar();
 
   // Reapply old settings
   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
 
   return ch;
 }
int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("teleop");
    // define publisher
    auto _pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
     // Set terminal settings for non-blocking input

    while(rclcpp::ok()){
        float x=0;
        float y=0;
        float yaw=0;
        geometry_msgs::msg::Twist twist;

        // get the pressed key
        
        key=getch();
        switch (key) {
            case 'i':
            x = .1;
            break;
            case 'k':
            x = -.1;
            break;
            case 'j':
            yaw = 1.0;
            break;
            case 'l':
            yaw = -1.0;
            break;
            // Add more cases as needed
        }
        twist.linear.x=x;
        twist.linear.y=y;
        twist.angular.z=yaw;
        _pub->publish(twist);
        rclcpp::spin_some(node);  
    }
    return 0;
}