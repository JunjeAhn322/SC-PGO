// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

std::shared_ptr<rclcpp::Node> n;

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "kitti_helper");
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("kitti_");

    std::string dataset_folder, sequence_number, output_bag_file;
    // n.getParam("dataset_folder", dataset_folder);
    n->declare_parameter<std::string>("dataset_folder", dataset_folder);
    n->get_parameter("dataset_folder", dataset_folder);
    // n.getParam("sequence_number", sequence_number);
    n->declare_parameter<std::string>("sequence_number", sequence_number);
    n->get_parameter("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    // n.getParam("to_bag", to_bag);
    n->declare_parameter<bool>("to_bag", to_bag);
    n->get_parameter("to_bag", to_bag);
    if (to_bag)
        // n.getParam("output_bag_file", output_bag_file);
        n->declare_parameter<std::string>("output_bag_file", output_bag_file);
        n->get_parameter("output_bag_file", output_bag_file);
    int publish_delay;
    // n.getParam("publish_delay", publish_delay);
    n->declare_parameter<int>("publish_delay", publish_delay);
    n->get_parameter("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    // ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2);
    auto pub_laser_cloud = n->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    // ros::Publisher pubOdomGT = n.advertise<nav_msgs::msg::Odometry> ("/odometry_gt", 5);
    auto pubOdomGT = n->create_publisher<nav_msgs::msg::Odometry>("/odometry_gt", 5);
    nav_msgs::msg::Odometry odomGT;
    odomGT.header.frame_id = "camera_init";
    odomGT.child_frame_id = "/ground_truth";

    // ros::Publisher pubPathGT = n.advertise<nav_msgs::msg::Path> ("/path_gt", 5);
    auto pubPathGT = n->create_publisher<nav_msgs::msg::Path>("/path_gt", 5);
    nav_msgs::msg::Path pathGT;
    pathGT.header.frame_id = "camera_init";

    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);
    
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    // ros::Rate r(10.0 / publish_delay);
    rclcpp::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && rclcpp::ok())
    {
        float timestamp = stof(line);
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = rclcpp::Time(timestamp*10e9);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT->publish(odomGT);

        geometry_msgs::msg::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT->publish(pathGT);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::msg::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = rclcpp::Time(timestamp*10e9);
        laser_cloud_msg.header.frame_id = "camera_init";
        pub_laser_cloud->publish(laser_cloud_msg);

        sensor_msgs::msg::Image::SharedPtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        line_num ++;
        r.sleep();
    }
    std::cout << "Done \n";


    return 0;
}