/**
 * pointcloud_merger.cpp
 * 
 * M300 LiDAR (360°, 수평)와 CameraSDK depth 카메라 (전방 120°×80°)의
 * 포인트클라우드를 합쳐서 하나의 토픽으로 발행하는 노드.
 * 
 * - M300: PointXYZI (x, y, z, intensity)
 * - Camera: PointXYZRGB (x, y, z, r, g, b) → intensity=0으로 변환
 * - 출력: PointXYZI → Fast-LIO가 구독
 * 
 * message_filters의 ApproximateTime 동기화를 사용하여
 * 두 센서의 서로 다른 주기(10Hz vs ~20Hz)를 처리합니다.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudMerger : public rclcpp::Node
{
public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;

    PointCloudMerger() : Node("pointcloud_merger")
    {
        // Parameters
        this->declare_parameter<std::string>("m300_topic", "/m300/pointcloud");
        this->declare_parameter<std::string>("camera_topic", "/lx_camera_node/LxCamera_Cloud");
        this->declare_parameter<std::string>("output_topic", "/pointcloud");
        this->declare_parameter<std::string>("output_frame", "base_scan");
        this->declare_parameter<double>("sync_tolerance", 0.1);

        std::string m300_topic, camera_topic, output_topic;
        this->get_parameter("m300_topic", m300_topic);
        this->get_parameter("camera_topic", camera_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("output_frame", output_frame_);
        this->get_parameter("sync_tolerance", sync_tolerance_);

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher
        pub_merged_ = this->create_publisher<PointCloud2>(output_topic, rclcpp::SensorDataQoS());

        // Subscribers with message_filters
        sub_m300_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(
            this, m300_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        sub_camera_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(
            this, camera_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        // ApproximateTime synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), *sub_m300_, *sub_camera_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
        sync_->registerCallback(
            std::bind(&PointCloudMerger::syncCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Also subscribe to M300 alone for fallback (camera might not always be available)
        sub_m300_alone_ = this->create_subscription<PointCloud2>(
            m300_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudMerger::m300AloneCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "PointCloudMerger started: M300[%s] + Camera[%s] → [%s] (frame: %s)",
                    m300_topic.c_str(), camera_topic.c_str(),
                    output_topic.c_str(), output_frame_.c_str());
    }

private:
    void syncCallback(const PointCloud2::ConstSharedPtr& m300_msg,
                      const PointCloud2::ConstSharedPtr& camera_msg)
    {
        last_sync_time_ = this->now();
        camera_available_ = true;

        // 1. Start with M300 cloud (already in base_scan frame)
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());

        // Convert M300 PointCloud2 → PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr m300_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*m300_msg, *m300_cloud);
        *merged += *m300_cloud;

        // 2. Transform Camera cloud to output_frame, then convert to XYZI
        try {
            // Transform camera cloud to base_scan frame
            PointCloud2 camera_transformed;
            if (camera_msg->header.frame_id != output_frame_) {
                auto transform = tf_buffer_->lookupTransform(
                    output_frame_, camera_msg->header.frame_id,
                    rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
                tf2::doTransform(*camera_msg, camera_transformed, transform);
            } else {
                camera_transformed = *camera_msg;
            }

            // Convert camera XYZRGB → XYZI (intensity=0)
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(camera_transformed, *camera_rgb);

            for (const auto& pt : camera_rgb->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                    continue;
                pcl::PointXYZI pi;
                pi.x = pt.x;
                pi.y = pt.y;
                pi.z = pt.z;
                pi.intensity = 0.0f;  // Camera has no intensity
                merged->push_back(pi);
            }

            RCLCPP_DEBUG(this->get_logger(),
                         "Merged: M300=%zu + Camera=%zu = %zu points",
                         m300_cloud->size(), camera_rgb->size(), merged->size());

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Camera TF failed, using M300 only: %s", ex.what());
        }

        // 3. Publish merged cloud
        PointCloud2 output;
        pcl::toROSMsg(*merged, output);
        output.header.stamp = m300_msg->header.stamp;
        output.header.frame_id = output_frame_;
        pub_merged_->publish(output);
    }

    void m300AloneCallback(const PointCloud2::ConstSharedPtr& msg)
    {
        // If camera sync is active, skip — syncCallback handles it
        if (camera_available_) {
            auto elapsed = (this->now() - last_sync_time_).seconds();
            if (elapsed < 2.0) {
                return;  // Camera sync is active, skip standalone
            }
            // Camera seems to have disconnected
            camera_available_ = false;
            RCLCPP_WARN(this->get_logger(), "Camera sync lost, falling back to M300 only");
        }

        // Fallback: republish M300 as-is
        auto output = std::make_unique<PointCloud2>(*msg);
        output->header.frame_id = output_frame_;
        pub_merged_->publish(std::move(output));
    }

    // Members
    std::string output_frame_;
    double sync_tolerance_;
    bool camera_available_ = false;
    rclcpp::Time last_sync_time_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<PointCloud2>::SharedPtr pub_merged_;
    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_m300_;
    std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub_camera_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_m300_alone_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
