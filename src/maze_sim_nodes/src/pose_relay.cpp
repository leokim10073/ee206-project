// Republishes the MuJoCo free-joint states as ground-truth poses in the world frame:
//   /sim/free_joint_states (FreeJointStateArray) -> /sim/ball/pose, /sim/board/pose (PoseStamped)
// C++ because it runs on every 100 Hz sample the vision and control code compare against.

#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>

class PoseRelay : public rclcpp::Node
{
public:
  PoseRelay() : Node("pose_relay")
  {
    for (const auto& [body, topic] : std::map<std::string, std::string>{ { "ball", "/sim/ball/pose" },
                                                                         { "board", "/sim/board/pose" } })
    {
      pubs_[body] = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    }
    sub_ = create_subscription<mujoco_ros2_control_msgs::msg::FreeJointStateArray>(
        "/sim/free_joint_states", 10,
        [this](mujoco_ros2_control_msgs::msg::FreeJointStateArray::UniquePtr msg) { relay(*msg); });
  }

private:
  void relay(mujoco_ros2_control_msgs::msg::FreeJointStateArray& msg)
  {
    for (auto& joint : msg.free_joints)
    {
      auto pub = pubs_.find(joint.name);
      if (pub == pubs_.end())
      {
        continue;
      }
      joint.pose.header.frame_id = "world";  // the plugin leaves it empty for the world frame
      pub->second->publish(joint.pose);
    }
  }

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubs_;
  rclcpp::Subscription<mujoco_ros2_control_msgs::msg::FreeJointStateArray>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRelay>());
  rclcpp::shutdown();
  return 0;
}
