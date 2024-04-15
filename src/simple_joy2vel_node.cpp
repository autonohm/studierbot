// generate a simple joy to vel node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>



class SimpleJoy2Vel : public rclcpp::Node
{
public:
  SimpleJoy2Vel()
  : Node("simple_joy2vel")
  {
    // create a subscriber to the topic joy with the type sensor_msgs::msg::Joy
    _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                            "joy", 
                            10, 
                            std::bind(&SimpleJoy2Vel::joyCallback, this, std::placeholders::_1)
                            );

    // create a publisher to the topic cmd_vel with the type geometry_msgs::msg::Twist
    _twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // create a new Twist message
    auto twist = std::make_shared<geometry_msgs::msg::Twist>();

    // set the linear velocity to the left joystick
    twist->linear.x = msg->axes[1] * 2;

    twist->linear.y = msg->axes[0] * 2;

    // set the angular velocity to the right joystick
    twist->angular.z = msg->axes[3] *3 ;

    // publish the Twist message
    _twist_pub->publish(*twist);
  }


private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twist_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleJoy2Vel>());
  rclcpp::shutdown();
  return 0;
}
