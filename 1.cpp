#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <deque>
#include <chrono>

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor"), frame_count_(0), start_time_(std::chrono::steady_clock::now())  // 初始化起始时间
     timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ImageProcessor::publishRvizMarker, this));
    
    {
        image_transport::ImageTransport it(get_node_base_interface());
        image_sub_ = it.subscribe("image_topic", rclcpp::SensorDataQoS(),
                                  std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));
        image_pub_ = it.advertise("processed_image_topic", rclcpp::SensorDataQoS());

        enemy_coordinate_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "enemy_coordinate_topic", rclcpp::SensorDataQoS(),
            std::bind(&ImageProcessor::enemyCoordinateCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

        // 计算帧率
        frame_count_++;
        auto current_time = std::chrono::steady_clock::now();  // 获取当前时间
        auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time_);  // 计算时间间隔
        if (elapsed_time.count() >= 1.0)  // 检查是否达到 1 秒
        {
            double fps = frame_count_ / elapsed_time.count();  // 计算帧率
            cv::putText(img, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

            frame_count_ = 0;
            start_time_ = current_time;  // 更新起始时间
        }

        sensor_msgs::msg::ImagePtr processed_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img).toImageMsg();
        image_pub_.publish(processed_img_msg);
    }

   void enemyCoordinateCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr enemy_coord_msg)
{
    try
    {
        // 左手坐标系到右手坐标系的转换
        geometry_msgs::msg::PointStamped transformed_coord;
        transformed_coord.point.x = -enemy_coord_msg->point.x;
        transformed_coord.point.y = enemy_coord_msg->point.y;
        transformed_coord.point.z = -enemy_coord_msg->point.z;

        // 从相机光心坐标系转换到 base_link 坐标系
        geometry_msgs::msg::PointStamped point_in_base_link;
        transformToBaseLink(transformed_coord, point_in_base_link);

        // 将转换后的坐标添加到队列
        enemy_coordinates_.push_back(point_in_base_link);
        if (enemy_coordinates_.size() > 10)
        {
            enemy_coordinates_.pop_front();
        }

        // 发布为 rviz2 箭头类型的 marker
        //...
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in enemy coordinate callback: %s", e.what());
    }
}
void publishRvizMarker()
{
    rviz2_common::msg::MarkerArray marker_array;
    for (size_t i = 0; i < enemy_coordinates_.size() - 1; ++i)
    {
        rviz2_common::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = now();
        marker.ns = "enemy_trajectory";
        marker.id = i;
        marker.type = rviz2_common::msg::Marker::ARROW;
        marker.action = rviz2_common::msg::Marker::ADD;

        marker.pose.position.x = enemy_coordinates_[i].point.x;
        marker.pose.position.y = enemy_coordinates_[i].point.y;
        marker.pose.position.z = enemy_coordinates_[i].point.z;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    rviz2_publisher_->publish(marker_array);
}
  void transformToBaseLink(const geometry_msgs::msg::PointStamped &point_in_camera, geometry_msgs::msg::PointStamped &point_in_base_link)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("base_link", point_in_camera.header.frame_id, rclcpp::Time(0));
        tf2::doTransform(point_in_camera, point_in_base_link, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
}

    int frame_count_;
    std::chrono::time_point<std::chrono::steady_clock> start_time_;  // 使用chrono的时间点类型
    image_transport::SubscriberFilter image_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr enemy_coordinate_sub_;
    std::deque<geometry_msgs::msg::PointStamped> enemy_coordinates_;
    
    if (timer_)
    {
        timer_->cancel();
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
