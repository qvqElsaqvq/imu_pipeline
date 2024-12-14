#include <imu_transformer/imu_transformer.hpp>
#include "imu_transformer/tf2_sensor_msgs.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace imu_transformer {
  ImuTransformer::ImuTransformer(const rclcpp::NodeOptions & options)
  : Node("imu_transformer", options),
    broadcaster_(this)
  {
    RCLCPP_INFO(get_logger(), "imu_transformer_node is created");
    this->declare_parameter("imu_frame_187", "livox_192_168_1_187");
    this->declare_parameter("imu_frame_104", "livox_192_168_1_104");
    this->declare_parameter("target_frame", "livox");
    this->declare_parameter("imu_187_trans", std::vector<double>{-0.05, 0.05, 0.0});
    this->declare_parameter("imu_187_rot", std::vector<double>{0.0, 0.0, 1.0, 0.0});
    this->declare_parameter("imu_104_trans", std::vector<double>{0.05, 0.05, 0.0});
    this->declare_parameter("imu_104_rot", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    this->declare_parameter("imu_in_topic1", "livox/imu_192_168_1_187");
    this->declare_parameter("imu_in_topic2", "livox/imu_192_168_1_104");
    this->declare_parameter("imu_out_topic", "livox/imu");
    this->declare_parameter("is_use_imu_104", true);

    this->get_parameter("imu_frame_187", imu_frame_187_);
    this->get_parameter("imu_frame_104", imu_frame_104_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("imu_187_trans", imu_187_trans_);
    this->get_parameter("imu_187_rot", imu_187_rot_);
    this->get_parameter("imu_104_trans", imu_104_trans_);
    this->get_parameter("imu_104_rot", imu_104_rot_);
    this->get_parameter("imu_in_topic1", imu_in_topic1_);
    this->get_parameter("imu_in_topic2", imu_in_topic2_);
    this->get_parameter("imu_out_topic", imu_out_topic_);
    this->get_parameter("is_use_imu_104", is_use_imu_104_);

    lidar_flag_ = 0;
    is_first_time_187_ = true;
    is_first_time_104_ = true;
    if(is_use_imu_104_) {
      initial_trans_ = imu_104_trans_;
      initial_rot_ = imu_104_rot_;
    }
    else {
      initial_trans_ = imu_187_trans_;
      initial_rot_ = imu_187_rot_;
    }

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_ros::TransformListener tf(*tf2_buffer_);

    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    // auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    //   this->get_node_base_interface(),
    //   this->get_node_timers_interface());
    // tf2_buffer_->setCreateTimerInterface(timer_interface);

    imu_pub_ = this->create_publisher<ImuMsg>(imu_out_topic_, 1000);
    lidar_flag_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("lidar_flag", 10,
      std::bind(&ImuTransformer::lidarFlagCallback, this, _1));

    // std::chrono::milliseconds buffer_timeout(1);

    // imu_sub1_.subscribe(this, imu_in_topic1_);
    // imu_sub2_.subscribe(this, imu_in_topic2_);
    // imu_filter1_ = std::make_shared<ImuFilter>(imu_sub1_, *tf2_buffer_, target_frame_,
    //   1000, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout);
    // imu_filter1_->registerCallback(&ImuTransformer::imuCallback1, this);
    // imu_filter2_ = std::make_shared<ImuFilter>(imu_sub2_, *tf2_buffer_, target_frame_,
    //   1000, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout);
    // imu_filter2_->registerCallback(&ImuTransformer::imuCallback2, this);

    imu_sub1_ = this->create_subscription<ImuMsg>(imu_in_topic1_, 10,
      std::bind(&ImuTransformer::imuCallback1, this, _1));
    imu_sub2_ = this->create_subscription<ImuMsg>(imu_in_topic2_, 10,
      std::bind(&ImuTransformer::imuCallback2, this, _1));

    timer_ = this->create_wall_timer( 0.005s, std::bind(&ImuTransformer::timerCallback, this));

    RCLCPP_INFO(get_logger(), "imu_transformer_node初始化完成");
  }

  void ImuTransformer::imuCallback1(const ImuMsg::SharedPtr imu_in)  //187
  {
    if(lidar_flag_ == 1) {
      if(is_first_time_187_) {
        try{
          transform_stamped_187_ = tf2_buffer_->lookupTransform(target_frame_, imu_frame_187_,
            imu_in->header.stamp, rclcpp::Duration::from_seconds(0.5));
        }catch (const tf2::TransformException & ex){
          RCLCPP_WARN(this->get_logger(), "IMU Transform failure %s\n", ex.what());
        }
        is_first_time_187_ = false;
      }
      ImuMsg::SharedPtr imu_out = std::make_shared<ImuMsg>();
      tf2::doTransform(*imu_in, *imu_out, transform_stamped_187_);

      // ImuMsg imu_out;
      // tf2_buffer_->transform(*imu_in, imu_out, target_frame_);

      imu_out->header.frame_id = target_frame_;
      imu_out->header.stamp = imu_in->header.stamp;
      imu_pub_->publish(*imu_out);
      //RCLCPP_INFO(get_logger(),"pub imu 187");
    }
  }

  void ImuTransformer::imuCallback2(const ImuMsg::SharedPtr imu_in)  //104
  {
    if(lidar_flag_ == 0 || lidar_flag_ == 2) {
      if(is_first_time_104_) {
        try{
          transform_stamped_104_ = tf2_buffer_->lookupTransform(target_frame_, imu_frame_104_,
            imu_in->header.stamp, rclcpp::Duration::from_seconds(0.5));
        }catch (const tf2::TransformException & ex){
          RCLCPP_WARN( this->get_logger(), "IMU Transform failure %s\n", ex.what());
        }
        is_first_time_104_ = false;
      }
      ImuMsg::SharedPtr imu_out = std::make_shared<ImuMsg>();
      tf2::doTransform(*imu_in, *imu_out, transform_stamped_104_);

      // ImuMsg imu_out;
      // tf2_buffer_->transform(*imu_in, imu_out, target_frame_);

      imu_out->header.frame_id = target_frame_;
      imu_out->header.stamp = imu_in->header.stamp;
      imu_pub_->publish(*imu_out);
      //RCLCPP_INFO(get_logger(),"pub imu 104");
    }
  }

  void ImuTransformer::timerCallback() {
    // 发布变换
    transform_mtx_.lock();
    transform_.header.stamp = now();
    transform_.header.frame_id = target_frame_;
    if(lidar_flag_ == 0 || lidar_flag_ == 2)
      transform_.child_frame_id = imu_frame_104_;
    else if(lidar_flag_ == 1)
      transform_.child_frame_id = imu_frame_187_;
    transform_.transform.translation.x = initial_trans_[0];
    transform_.transform.translation.y = initial_trans_[1];
    transform_.transform.translation.z = initial_trans_[2];
    transform_.transform.rotation.x = initial_rot_[0];
    transform_.transform.rotation.y = initial_rot_[1];
    transform_.transform.rotation.z = initial_rot_[2];
    transform_.transform.rotation.w = initial_rot_[3];
    broadcaster_.sendTransform(transform_);
     //RCLCPP_INFO(get_logger(), "translation: [ %f, %f, %f], rotation: [ %f, %f, %f, %f]", initial_trans_[0],
     // initial_trans_[1], initial_trans_[2], initial_rot_[0], initial_rot_[1], initial_rot_[2], initial_rot_[3]);
    transform_mtx_.unlock();
  }

  void ImuTransformer::lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    lidar_flag_ = msg->data;
    if(lidar_flag_ == 0 || lidar_flag_ == 2) {  //104
      initial_trans_ = imu_104_trans_;
      initial_rot_ = imu_104_rot_;
      RCLCPP_INFO(get_logger(),"--------------------------use imu 104--------------------------");
    }
    else if(lidar_flag_ == 1) {  //187
      initial_trans_ = imu_187_trans_;
      initial_rot_ = imu_187_rot_;
      RCLCPP_INFO(get_logger(),"--------------------------use imu 187--------------------------");
    }
    else {
      RCLCPP_WARN(get_logger(),"--------------------------lose lidar--------------------------");
    }
  }
  
  ImuTransformer::~ImuTransformer() {}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_transformer::ImuTransformer)
