#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class RadiomasterNode : public rclcpp::Node
{
public:
    RadiomasterNode() : Node("radiomaster_joystick_controller")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&RadiomasterNode::joy_callback, this, _1));
        
        this->declare_parameter<std::string>("rpyt_topic", "joystick_radiomaster/rpyt");
        this->declare_parameter<std::string>("gear_S1_topic", "joystick/gear_S1");
        this->declare_parameter<std::string>("switch_SC_topic", "joystick/switch_SC");
        this->declare_parameter<std::string>("switch_SB_topic", "joystick/switch_SB");
        this->declare_parameter<std::string>("button_SD_topic", "joystick/SD");
        this->declare_parameter<std::string>("button_SA_topic", "joystick/SA");
        this->declare_parameter<std::string>("button_SE_topic", "joystick/SE");
        this->declare_parameter<std::string>("button_SAB_topic", "joystick/SAB");
        std::string rpyt_topic = this->get_parameter("rpyt_topic").as_string();
        std::string gear_S1_topic = this->get_parameter("gear_S1_topic").as_string();
        std::string switch_SC_topic = this->get_parameter("switch_SC_topic").as_string();
        std::string switch_SB_topic = this->get_parameter("switch_SB_topic").as_string();
        std::string button_SD_topic = this->get_parameter("button_SD_topic").as_string();
        std::string button_SA_topic = this->get_parameter("button_SA_topic").as_string();
        std::string button_SE_topic = this->get_parameter("button_SE_topic").as_string();
        std::string button_SAB_topic = this->get_parameter("button_SAB_topic").as_string();
            
        rpyt_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(rpyt_topic, 10);
        gear_S1_pub_ = this->create_publisher<std_msgs::msg::Float32>(gear_S1_topic, 10);
        switch_SC_pub_ = this->create_publisher<std_msgs::msg::Float32>(switch_SC_topic, 10);
        switch_SB_pub_ = this->create_publisher<std_msgs::msg::Float32>(switch_SB_topic, 10);
        button_SD_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_SD_topic, 10);
        button_SA_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_SA_topic, 10);
        button_SE_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_SE_topic, 10);
        button_SAB_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_SAB_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Radiomaster Joystick Controller Node Started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Assi principali RPYT -> assi 0,1,2,3
        if (msg->axes.size() >= 4) {
            std_msgs::msg::Float32MultiArray rpyt_msg;
            rpyt_msg.data = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
            rpyt_pub_->publish(rpyt_msg);
        }

        // Altri assi -> 4,5,6
        if (msg->axes.size() >= 7) {
            publish_axis(gear_S1_pub_, msg->axes[4]);
            publish_axis(switch_SC_pub_, msg->axes[5]);
            publish_axis(switch_SB_pub_, msg->axes[6]);
        }

        // Pulsanti -> 0,1,2,3
        if (msg->buttons.size() >= 4) {
            publish_button(button_SE_pub_, msg->buttons[0]);
            publish_button(button_SA_pub_, msg->buttons[1]);
            publish_button(button_SD_pub_, msg->buttons[2]);
            publish_button(button_SAB_pub_, msg->buttons[3]);
        }
    }

    void publish_axis(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, float value)
    {
        std_msgs::msg::Float32 msg;
        msg.data = value;
        pub->publish(msg);
    }

    void publish_button(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, int button_value)
    {
        std_msgs::msg::Bool msg;
        msg.data = (button_value != 0);
        pub->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpyt_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gear_S1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr switch_SC_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr switch_SB_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_SD_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_SA_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_SE_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_SAB_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadiomasterNode>());
    rclcpp::shutdown();
    return 0;
}
