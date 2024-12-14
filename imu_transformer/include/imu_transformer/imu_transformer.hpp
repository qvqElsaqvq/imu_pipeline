#ifndef IMU_TRANSFORMER_IMU_TRANSFORMER_HPP
#define IMU_TRANSFORMER_IMU_TRANSFORMER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "std_msgs/msg/int32.hpp"

#include <string>

namespace imu_transformer
{
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef tf2_ros::MessageFilter<ImuMsg> ImuFilter;

  class ImuTransformer : public rclcpp::Node
  {

  public:
    explicit ImuTransformer(const rclcpp::NodeOptions &);
    ~ImuTransformer();

  private:
    std::vector<double> imu_187_trans_;
    std::vector<double> imu_187_rot_;
    std::vector<double> imu_104_trans_;
    std::vector<double> imu_104_rot_;
    std::vector<double> initial_trans_;
    std::vector<double> initial_rot_;

    bool is_use_imu_104_;
    int lidar_flag_;
    bool is_first_time_187_;
    bool is_first_time_104_;

    geometry_msgs::msg::TransformStamped transform_stamped_187_;
    geometry_msgs::msg::TransformStamped transform_stamped_104_;
    geometry_msgs::msg::TransformStamped transform_;
    std::mutex transform_mtx_;
    tf2_ros::TransformBroadcaster broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::string imu_frame_187_;
    std::string imu_frame_104_;
    std::string imu_frame_;
    std::string target_frame_;

    std::string imu_in_topic1_;
    std::string imu_in_topic2_;
    std::string imu_out_topic_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

    // ImuSubscriber imu_sub1_;
    // std::shared_ptr<ImuFilter> imu_filter1_;
    // ImuSubscriber imu_sub2_;
    // std::shared_ptr<ImuFilter> imu_filter2_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub1_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub2_;
    rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lidar_flag_subscriber_;

    void imuCallback1(const ImuMsg::SharedPtr imu_in);
    void imuCallback2(const ImuMsg::SharedPtr imu_in);
    void lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void timerCallback();
  };

}  // namespace imu_transformer

#endif  // IMU_TRANSFORMER_IMU_TRANSFORMER_HPP
