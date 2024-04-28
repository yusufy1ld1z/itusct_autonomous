#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>


class JoyControllerNode : public rclcpp::Node
{
public:
    JoyControllerNode() : Node("joy_controller") {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&JoyControllerNode::pose_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        timer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&JoyControllerNode::timerCallback, this));
        timer->cancel(); // Cancel the timer initially, so it will not be triggered until the first event.

        RCLCPP_INFO(this->get_logger(), "Key Bindings:");
        RCLCPP_INFO(this->get_logger(), "- Right Back Trigger: Control linear velocity of the turtle.");
        RCLCPP_INFO(this->get_logger(), "- Left Horizontal Stick: Control angular velocity of the turtle.");
        RCLCPP_INFO(this->get_logger(), "- Button A: Clears turtlesim background and sets color.");
        RCLCPP_INFO(this->get_logger(), "- Button B: Resets turtlesim to start configuration and sets background color.");
        RCLCPP_INFO(this->get_logger(), "- Button X: Spawns a turtle at current position and orientation.");
        RCLCPP_INFO(this->get_logger(), "- Button Y: Sets pen color, width, and turns pen on/off.");
        RCLCPP_INFO(this->get_logger(), "- Button Start or Left Stick Button(LSB): Exits the program.");
    }
private:
    /**
     * @brief Callback function for the Joy message.
     *
     * This function is called whenever a Joy message is received. With the explicit controls the durability of the program is increased.
     * Each event is only triggered again after a specific time interval, ensuring the correctness of the program. This is done by the timerCallback function.
     * Necessary services like clear, reset, spawn, and set_pen are called according to the button pressed, and cmd_vel is published according to the joystick.
     *
     * @param msg Joy message
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
       if(msg->buttons[0] == 1 && lastButton != 0) { // A button Clear
            auto client = rclcpp::Node::create_client<std_srvs::srv::Empty>("clear");
            auto request = client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
            lastButton = 0;
            // timer->cancel(); // It is allowed to cancel the timer which already cancelled, but it is unnecessary since timer callback cancels itself.
            timer->reset(); // Reset the timer. so it will execute after a 0.3 seconds. This ensures that the lastButton will be reset after 0.3 seconds.
                            // until then, the lastButton will be the same, and same event will not be triggered again because of the if statement.
        }
        else if(msg->buttons[1] == 1 && lastButton != 1){ // B button Reset
            auto client = rclcpp::Node::create_client<std_srvs::srv::Empty>("reset");
            auto request = client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
            lastButton = 1;
            timer->reset();
        }
        else if(msg->buttons[2] == 1 && lastButton != 2){ // X Button Spawn
            auto client = rclcpp::Node::create_client<turtlesim::srv::Spawn>("spawn");
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = turtle_x; // Fill the request with the current turtle pose
            request->y = turtle_y;
            request->theta = turtle_theta;
            auto result = client->async_send_request(request);
            lastButton = 2;
            timer->reset();
        }
        else if(msg->buttons[3] == 1 && lastButton != 3){ // Y button setpen
            auto client = rclcpp::Node::create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
            auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
            request->r = 0xb3;
            request->g = 0xb8;
            request->b = 0xff;
            request->width = 3;
            request->off = !is_pen_on;
            is_pen_on = !is_pen_on;
            auto result = client->async_send_request(request);
            lastButton = 3;
            timer->reset();
        }
        else if(msg->buttons[9] == 1) // Start Button or Left Stick Button
            rclcpp::shutdown();
        else { // Joystick
            geometry_msgs::msg::Twist msg_ = geometry_msgs::msg::Twist();
            msg_.linear.x = (msg->axes[5] + -1) * -2; // Linear and angular acceleration is polished
            msg_.angular.z = msg->axes[0] * 2.5;
            publisher_->publish(msg_);
        }
        
    }
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        turtle_x = msg->x;
        turtle_y = msg->y;
        turtle_theta = msg->theta;
    }

    void timerCallback(){
        lastButton = -1; // Reset the lastButton so that same event can be triggered again.
        timer->cancel();
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    float turtle_x, turtle_y, turtle_theta;
    bool is_pen_on = false;
    int lastButton = -1;
    rclcpp::TimerBase::SharedPtr timer;

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
}