#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>


class MarkerPublisher {
 private:
    ros::NodeHandle nh;

    double marker_size;
    std::string optical_frame;
    std::unordered_set<int> id_filter;

    ros::Publisher pub_vis;
    ros::Publisher pub_debug_image;
    ros::Publisher pub_debug_rejected;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    sensor_msgs::CameraInfoConstPtr        camera_info;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Ptr<cv::aruco::Dictionary>         marker_dict;

 public:
    void cb_image(sensor_msgs::ImageConstPtr msg) {
        if (!camera_info)
            return;
        
        auto bridge_image = cv_bridge::toCvShare(msg);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_corners;

        cv::aruco::detectMarkers(bridge_image->image, marker_dict, marker_corners, marker_ids, detector_params, rejected_corners);

        if (id_filter.size() > 0) {
            decltype(marker_ids) filtered_ids;
            decltype(marker_corners) filtered_corners;

            for (size_t i = 0; i < marker_ids.size(); i++) {
                if (id_filter.find(marker_ids[i]) != id_filter.end()) {
                    filtered_ids.push_back(marker_ids[i]);
                    filtered_corners.push_back(marker_corners[i]);
                }
            }

            marker_ids     = filtered_ids;
            marker_corners = filtered_corners;
        }


        if (!marker_ids.empty()) {
            cv::Mat camera_matrix(cv::Size(3, 4), CV_64F, const_cast<void*>((const void*)camera_info->P.data()));
            cv::Mat dist_coeff(1, (int)camera_info->D.size(), CV_64F, const_cast<void*>((const void*)camera_info->D.data()));

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size, 
                                                 camera_matrix, dist_coeff, rvecs, tvecs);
        
    
            cv_bridge::CvImagePtr debug_img;
            if (pub_debug_image.getNumSubscribers() > 0) {
                debug_img = cv_bridge::toCvCopy(msg);
                cv::aruco::drawDetectedMarkers(debug_img->image, marker_corners, marker_ids);
            }


            std::vector<geometry_msgs::TransformStamped> tfs_to_publish;
            // Angle of rotation
            for (size_t i = 0; i < marker_ids.size(); i++) {
                if (debug_img)
                    cv::aruco::drawAxis(debug_img->image, camera_matrix, dist_coeff, rvecs[i], tvecs[i], 0.1);
                
                double angle = sqrt(rvecs[i].ddot(rvecs[i]));
                double quat_angle = cos(0);
                cv::Vec3d quat_axis = cv::Vec3d(0.0);
                if (angle < 1e-4) {
                    auto rect_axis  = rvecs[0] / angle;
                    quat_axis  = rect_axis * sin(angle * 0.5);
                    quat_angle = cos(angle * 0.5);
                }

                std::stringstream ss;
                ss << marker_ids[i];

                geometry_msgs::TransformStamped tf_stamped;
                tf_stamped.header.stamp = ros::Time::now();
                tf_stamped.header.frame_id = optical_frame;
                tf_stamped.child_frame_id  = ss.str();
                tf_stamped.transform.translation.x = tvecs[i][0];
                tf_stamped.transform.translation.y = tvecs[i][1];
                tf_stamped.transform.translation.z = tvecs[i][2];
                tf_stamped.transform.rotation.x = quat_axis[0];
                tf_stamped.transform.rotation.y = quat_axis[1];
                tf_stamped.transform.rotation.z = quat_axis[2];
                tf_stamped.transform.rotation.w = quat_angle;
                tfs_to_publish.push_back(tf_stamped);
            }

            tf_broadcaster.sendTransform(tfs_to_publish);
            
            if (debug_img) {
                debug_img->header.stamp = ros::Time::now();
                debug_img->header.frame_id = optical_frame;
                pub_debug_image.publish(debug_img->toImageMsg());
            }

            if (pub_debug_rejected.getNumSubscribers() > 0) {
                auto debug_rejected_img = cv_bridge::toCvCopy(msg);
                cv::aruco::drawDetectedMarkers(debug_rejected_img->image, 
                                               rejected_corners, 
                                               cv::noArray(), 
                                               cv::Scalar(255, 0, 0));
                debug_rejected_img->header.stamp    = ros::Time::now();
                debug_rejected_img->header.frame_id = optical_frame;
                pub_debug_rejected.publish(debug_rejected_img->toImageMsg());
            }
        }
    }
    
    void cb_camera_info(sensor_msgs::CameraInfoConstPtr msg) {
        if (!camera_info) {
            camera_info = msg;
            optical_frame = camera_info->header.frame_id;
        }
    }
    
    MarkerPublisher(double marker_size,
                    cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name,
                    const std::unordered_set<int>& id_filter)
        : id_filter(id_filter) {
        detector_params = cv::aruco::DetectorParameters::create();
        marker_dict = cv::aruco::getPredefinedDictionary(dict_name);

        // pub_vis            = nh.advertise<visualization_msgs::Marker>("~visualization", 5);
        pub_debug_image    = nh.advertise<sensor_msgs::Image>("~debug_markers", 1);
        pub_debug_rejected = nh.advertise<sensor_msgs::Image>("~debug_rejected", 1);

        nh.subscribe("/camera_info", 1, &MarkerPublisher::cb_camera_info, this);
        nh.subscribe("/image_rect",  1, &MarkerPublisher::cb_image,       this);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "another_aruco_detector");
    ros::NodeHandle nh;

    std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_names {
        {"DICT_4X4_50",         cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100",        cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250",        cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000",       cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50",         cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100",        cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250",        cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000",       cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50",         cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100",        cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250",        cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000",       cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50",         cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100",        cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250",        cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000",       cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
    };

    double marker_size;
    nh.param<double>("~marker_size", marker_size, 0.08);
    std::string marker_dict;
    cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name = cv::aruco::DICT_4X4_50;
    if (nh.getParam("~marker_dict", marker_dict)) {
        auto it = dict_names.find(marker_dict);
        if (it == dict_names.end()) {
            std::cerr << "Unknown dict name \"" << marker_dict << "\" terminating node" << std::endl;
            return -1;
        }

        dict_name = it->second;
    }

    std::vector<int> id_filter_list;
    nh.getParam("~id_filter", id_filter_list);
    std::unordered_set<int> id_filter(id_filter_list.begin(), id_filter_list.end());

    MarkerPublisher(marker_size, dict_name, id_filter);

    ros::spin();
}