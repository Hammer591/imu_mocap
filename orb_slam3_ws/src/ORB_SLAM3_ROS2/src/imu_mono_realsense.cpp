#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// nav2_map_server removed (not available in this environment)

#include <chrono>
#include <filesystem>
#include <sstream>

#include <cv_bridge/cv_bridge.h>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node {
public:
  // Ensure aligned allocation for Eigen/Sophus member types (Tcw_, etc.)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH) +
                           "/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
      inertial_ba1_(false), inertial_ba2_(false)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);

    // get parameters
    sensor_type_param = get_parameter("sensor_type").as_string();
    use_pangolin = get_parameter("use_pangolin").as_bool();

    // define callback groups
    image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slam_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;

    // set the sensor type based on parameter
    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular") {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path =
        std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml";
    } else if (sensor_type_param == "imu-monocular") {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/config/Monocular-Inertial/RealSense_D435i.yaml";
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // Use explicit new instead of make_shared to ensure Eigen alignment
    // (make_shared bypasses EIGEN_MAKE_ALIGNED_OPERATOR_NEW)
    orb_slam3_system_ = std::shared_ptr<ORB_SLAM3::System>(
      new ORB_SLAM3::System(vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0));

    // create publishers
    live_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("live_point_cloud", 10);
    pose_array_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
    live_occupancy_grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("live_occupancy_grid", 10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("orb_odom", 10);
    orb_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("/orb_camera/image", 10);
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>("/orb_camera/imu", 10);

    // create subscriptions
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    image_sub = create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", image_qos,
      std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
    imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/imu", imu_qos,
      std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

    // tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer = create_wall_timer(
      100ms, std::bind(&ImuMonoRealSense::timer_callback, this),
      timer_callback_group_);

    timestamp_ = generate_timestamp_string();

    std::string path = std::string(PROJECT_PATH) + "/output/" + timestamp_;
    if (!std::filesystem::create_directory(path)) {
      std::cout << "Failed to create output directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/cloud")) {
      std::cout << "Failed to create cloud directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/grid")) {
      std::cout << "Failed to create grid directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/video")) {
      std::cout << "Failed to create images directory" << std::endl;
      return;
    }

    rclcpp::on_shutdown([this]() {
      video_writer_.release();
      if (live_pcl_cloud_.points.size() > 0) {
        pcl::io::savePCDFileBinary(std::string(PROJECT_PATH) + "/output/" +
                                     timestamp_ + "/cloud/" + timestamp_ + ".pcd",
                                   live_pcl_cloud_);
      }
    });

    initialize_variables();

    std::string orb_slam_video_path = std::string(PROJECT_PATH) + "/output/" +
                                      timestamp_ + "/video/" + timestamp_ +
                                      ".mp4";
    RCLCPP_INFO_STREAM(get_logger(), "Video path: " << orb_slam_video_path);
    video_writer_.open(orb_slam_video_path,
                       cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30,
                       cv::Size(640, 500));

    if (!video_writer_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Error opening video writer");
      rclcpp::shutdown();
    }
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    sor.filter(*sor_cloud);

    // radius outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(sor_cloud);
    radius_outlier.setRadiusSearch(
      0.1); // Adjust based on spacing in the point cloud
    radius_outlier.setMinNeighborsInRadius(
      5); // Increase for more aggressive outlier removal
    radius_outlier.filter(*radius_cloud);

    return radius_cloud;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // calculate the centroid
    Eigen::Matrix<float, 4, 1> centroid;
    pcl::ConstCloudIterator<pcl::PointXYZ> cloud_iterator(*cloud);
    pcl::compute3DCentroid(cloud_iterator, centroid);

    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();

    for (const auto &point : cloud->points) {
      if (point.x > max_x) {
        max_x = point.x;
      }
      if (point.y > max_y) {
        max_y = point.y;
      }
      if (point.x < min_x) {
        min_x = point.x;
      }
      if (point.y < min_y) {
        min_y = point.y;
      }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid =
      std::make_shared<nav_msgs::msg::OccupancyGrid>();
    cloud->width = cloud->points.size();
    occupancy_grid->header.frame_id = "live_map";
    occupancy_grid->header.stamp = get_clock()->now();
    occupancy_grid->info.resolution = 0.05;
    occupancy_grid->info.width =
      std::abs(max_x - min_x) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.height =
      std::abs(max_y - min_y) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.origin.position.x = min_x;
    occupancy_grid->info.origin.position.y = min_y;
    occupancy_grid->info.origin.position.z = 0;
    occupancy_grid->info.origin.orientation.x = 0;
    occupancy_grid->info.origin.orientation.y = 0;
    occupancy_grid->info.origin.orientation.z = 0;
    occupancy_grid->info.origin.orientation.w = 1;
    occupancy_grid->data.resize(
      occupancy_grid->info.width * occupancy_grid->info.height, 0);
    for (const auto &point : cloud->points) {
      int x = (point.x - min_x) / occupancy_grid->info.resolution;
      int y = (point.y - min_y) / occupancy_grid->info.resolution;
      int index = y * occupancy_grid->info.width + x;
      occupancy_grid->data.at(index) = 100;
    }
    return occupancy_grid;
  }

  void initialize_variables()
  {
    pose_array_ = geometry_msgs::msg::PoseArray();
    pose_array_.header.frame_id = "live_map";

    live_pcl_cloud_msg_ = sensor_msgs::msg::PointCloud2();
    live_pcl_cloud_msg_.header.frame_id = "live_map";

    live_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      // Return an empty grayscale image as fallback
      return cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    }

    if (cv_ptr) {
      return cv_ptr->image.clone();
    } else {
      RCLCPP_ERROR(get_logger(), "cv_ptr is null after cv_bridge conversion");
      return cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    rclcpp::Time time_now = get_clock()->now();

    sensor_msgs::msg::Image::SharedPtr msg_out =
      std::make_shared<sensor_msgs::msg::Image>(*msg);
    msg_out->header.stamp = time_now;
    msg_out->header.frame_id = "base_link";
    orb_image_publisher_->publish(*msg_out);

    img_buf_.push(msg);

    // begin to empty the img_buf_ queue, which is full of other queues
    while (!img_buf_.empty()) {
      // grab the oldest image
      auto imgPtr = img_buf_.front();
      img_buf_.pop();

      cv::Mat imageFrame = get_image(imgPtr);
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      // package all the imu data for this image for orbslam3 to process
      buf_mutex_imu_.lock();
      while (!imu_buf_.empty()) {
        auto imuPtr = imu_buf_.front();
        imu_buf_.pop();
        double tIMU =
          imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;

        cv::Point3f acc(imuPtr->linear_acceleration.x,
                        imuPtr->linear_acceleration.y,
                        imuPtr->linear_acceleration.z);
        cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                        imuPtr->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
      }

      buf_mutex_imu_.unlock();

      try {
        Sophus::SE3f Tcw;
        bool bTracked = false;
        if (sensor_type_param == "monocular") {
          Tcw = orb_slam3_system_->TrackMonocular(imageFrame, tImage);
          bTracked = true;
        } else {
          // 官方保护：IMU 预积分数据不足（<=1 帧）时跳过该帧。
          // 空 vImuMeas 传入 TrackMonocular 会让 ORB-SLAM3 内部
          // 以空预积分初始化，触发 inertial optimization SIGSEGV。
          if (vImuMeas.size() > 1) {
            Tcw = orb_slam3_system_->TrackMonocular(imageFrame, tImage, vImuMeas);
            bTracked = true;
          }
        }
        // 位姿更新条件（参考 Robo-Dude/ROS2_ORB-SLAM3_Odometry）：
        //   state>=2（RECENTLY_LOST 或 OK）时也更新 —— 单目模式视觉易短暂丢失，
        //   此时 ORB 仍返回预测位姿，发布它保证轨迹连续（可能有轻微漂移）。
        //   丢弃 identity/零位姿，避免失败帧污染轨迹。
        if (bTracked && orb_slam3_system_->GetTrackingState() >= 2 &&
            Tcw.translation().norm() > 1e-4) {
          // Twc = 世界→相机（相机在世界的位姿）
          Sophus::SE3f Twc = Tcw.inverse();
          // 增量累加：用帧间 delta 累加，避免直接覆盖导致轨迹跳变
          if (!has_prev_pose_) {
            prev_pose_ = Twc;
            has_prev_pose_ = true;
          }
          Sophus::SE3f delta = prev_pose_.inverse() * Twc;
          accumulated_pose_ = accumulated_pose_ * delta;
          prev_pose_ = Twc;
          Twc_ = accumulated_pose_;
          have_valid_pose_ = true;
        }
        cv::Mat pretty_frame = orb_slam3_system_->getPrettyFrame();
        video_writer_.write(pretty_frame);
        // DEBUG: 每 30 帧打印一次跟踪状态，确认 image_callback 持续工作
        {
          static int dbg_frame = 0;
          if ((dbg_frame++ % 30) == 0) {
            int ts = orb_slam3_system_->GetTrackingState();
            std::cout << "[DBG] img_cb state=" << ts
                      << " valid=" << have_valid_pose_
                      << " t=" << tImage << std::endl;
          }
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    buf_mutex_imu_.lock();
    sensor_msgs::msg::Imu msg_out = msg;
    msg_out.header.stamp = get_clock()->now();
    msg_out.header.frame_id = "base_link";
    // imu_publisher_->publish(msg_out);
    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z)) {
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      imu_buf_.push(msg_ptr);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    buf_mutex_imu_.unlock();
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    geometry_msgs::msg::Pose pose;
    // 发布条件：纯单目模式直接发布；IMU 模式需初始化完成。
    // 要求 have_valid_pose_（至少成功跟踪过一次），防止发布 identity。
    if ((orb_slam3_system_->isImuInitialized() ||
         sensor_type_param == "monocular") &&
        have_valid_pose_) {
      rclcpp::Time time_now = get_clock()->now();
      // Twc_ 已是 image_callback 增量累加后的世界→相机位姿
      Sophus::SE3f Twc = Twc_;

      // 坐标系转换：ORB-SLAM3 (X=右,Y=下,Z=前) → ROS REP-103 (X=前,Y=左,Z=上)
      // 参考 Robo-Dude/ROS2_ORB-SLAM3_Odometry
      Eigen::Matrix3f R_slam_to_ros;
      R_slam_to_ros << 0,  0, 1,
                      -1,  0, 0,
                      0,  -1, 0;
      Eigen::Vector3f t_ros = R_slam_to_ros * Twc.translation();
      Eigen::Matrix3f R_ros = R_slam_to_ros * Twc.rotationMatrix() * R_slam_to_ros.transpose();
      Eigen::Quaternionf q_ros(R_ros);
      q_ros.normalize();

      // 发布 TF odom → base_link
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = time_now;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = t_ros.x();
      odom_tf.transform.translation.y = t_ros.y();
      odom_tf.transform.translation.z = t_ros.z();
      odom_tf.transform.rotation.x = q_ros.x();
      odom_tf.transform.rotation.y = q_ros.y();
      odom_tf.transform.rotation.z = q_ros.z();
      odom_tf.transform.rotation.w = q_ros.w();
      tf_broadcaster->sendTransform(odom_tf);

      // 发布 /orb_odom
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time_now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = t_ros.x();
      odom.pose.pose.position.y = t_ros.y();
      odom.pose.pose.position.z = t_ros.z();
      odom.pose.pose.orientation.x = q_ros.x();
      odom.pose.pose.orientation.y = q_ros.y();
      odom.pose.pose.orientation.z = q_ros.z();
      odom.pose.pose.orientation.w = q_ros.w();
      odom_publisher_->publish(odom);

      // 发布 /pose_array（累积轨迹）
      geometry_msgs::msg::Pose pose;
      pose.position.x = t_ros.x();
      pose.position.y = t_ros.y();
      pose.position.z = t_ros.z();
      pose.orientation.x = q_ros.x();
      pose.orientation.y = q_ros.y();
      pose.orientation.z = q_ros.z();
      pose.orientation.w = q_ros.w();
      pose_array_.header.stamp = time_now;
      pose_array_.poses.push_back(pose);
      pose_array_publisher_->publish(pose_array_);

      geometry_msgs::msg::TransformStamped point_cloud_tf;
      point_cloud_tf.header.stamp = time_now;
      point_cloud_tf.header.frame_id = "map";
      point_cloud_tf.child_frame_id = "point_cloud";
      tf_broadcaster->sendTransform(point_cloud_tf);

      geometry_msgs::msg::TransformStamped live_map_tf;
      live_map_tf.header.stamp = time_now;
      live_map_tf.header.frame_id = "map";
      live_map_tf.child_frame_id = "live_map";
      tf_broadcaster->sendTransform(live_map_tf);

      live_pcl_cloud_ = orb_slam3_system_->GetMapPCL();

      pcl::PointCloud<pcl::PointXYZ>::Ptr live_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(live_pcl_cloud_);

      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      filtered_cloud_ptr = filter_point_cloud(live_ptr);

      filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();
      pcl::toROSMsg(*filtered_cloud_ptr, live_pcl_cloud_msg_);

      // this can go
      // live_occupancy_grid_ = point_cloud_to_occupancy_grid(filtered_cloud_ptr);
      // live_occupancy_grid_->header.stamp = time_now;
      // live_occupancy_grid_->header.frame_id = "live_map";
      // live_occupancy_grid_publisher_->publish(*live_occupancy_grid_);

      live_pcl_cloud_msg_.header.stamp = time_now;
      live_pcl_cloud_msg_.header.frame_id = "live_map";
      live_point_cloud_publisher_->publish(live_pcl_cloud_msg_);
    } else {
      // RCLCPP_INFO_STREAM(get_logger(), "IMU not initialized");
      initialize_variables();
    }
    if (!inertial_ba1_ && orb_slam3_system_->GetInertialBA1()) {
      inertial_ba1_ = true;
      initialize_variables();
      RCLCPP_INFO(get_logger(), "Inertial BA1 complete");
    }

    if (!inertial_ba2_ && orb_slam3_system_->GetInertialBA2()) {
      inertial_ba2_ = true;
      initialize_variables();
      RCLCPP_INFO(get_logger(), "Inertial BA2 complete");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    live_point_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    pose_array_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    live_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orb_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr slam_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  sensor_msgs::msg::Imu imu_msg;
  geometry_msgs::msg::PoseArray pose_array_;

  std::string sensor_type_param;
  bool use_pangolin;

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
  std::mutex buf_mutex_imu_, buf_mutex_img_, orbslam3_mutex_,
    live_pcl_cloud_mutex_;

  std::shared_ptr<ORB_SLAM3::System> orb_slam3_system_;
  std::string vocabulary_file_path;
  std::string settings_file_path;

  sensor_msgs::msg::PointCloud2 live_pcl_cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ> live_pcl_cloud_;
  nav_msgs::msg::OccupancyGrid::SharedPtr live_occupancy_grid_;

  bool inertial_ba1_;
  bool inertial_ba2_;
  bool have_valid_pose_ = false;
  // 增量累加位姿（参考 Robo-Dude/ROS2_ORB-SLAM3_Odometry）：
  //   prev_pose_ 上一帧世界→相机，accumulated_pose_ 累积位姿，Twc_ 供 timer 发布
  bool has_prev_pose_ = false;
  Sophus::SE3f prev_pose_;
  Sophus::SE3f accumulated_pose_;
  Sophus::SE3f Twc_;

  cv::VideoWriter video_writer_;
  std::string timestamp_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // Use explicit new instead of make_shared to ensure Eigen alignment for
  // members like Sophus::SE3f Tcw_ (make_shared can misalign over-aligned types)
  rclcpp::spin(std::shared_ptr<ImuMonoRealSense>(new ImuMonoRealSense()));
  rclcpp::shutdown();
  return 0;
}
