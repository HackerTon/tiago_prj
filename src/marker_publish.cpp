/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file marker_publish.cpp
* @author Bence Magyar
* @date June 2014
* @brief Modified copy of simple_single.cpp to publish all markers visible
* (modified by Josh Langsfeld, 2014)
*/

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>

namespace utils {
aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo &cam_info,
                                                     bool useRectifiedParameters) {
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if (useRectifiedParameters) {
        cameraMatrix.setTo(0);
        cameraMatrix.at<double>(0, 0) = cam_info.P[0];
        cameraMatrix.at<double>(0, 1) = cam_info.P[1];
        cameraMatrix.at<double>(0, 2) = cam_info.P[2];
        cameraMatrix.at<double>(1, 0) = cam_info.P[4];
        cameraMatrix.at<double>(1, 1) = cam_info.P[5];
        cameraMatrix.at<double>(1, 2) = cam_info.P[6];
        cameraMatrix.at<double>(2, 0) = cam_info.P[8];
        cameraMatrix.at<double>(2, 1) = cam_info.P[9];
        cameraMatrix.at<double>(2, 2) = cam_info.P[10];

        for (int i = 0; i < 4; ++i)
            distorsionCoeff.at<double>(i, 0) = 0;
    } else {
        for (int i = 0; i < 9; ++i)
            cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.K[i];

        if (cam_info.D.size() == 4) {
            for (int i = 0; i < 4; ++i)
                distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
        } else {
            ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
            for (int i = 0; i < 4; ++i)
                distorsionCoeff.at<double>(i, 0) = 0;
        }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf::Transform arucoMarker2Tf(const aruco::Marker &marker, bool rotate_marker_axis) {
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Mat Rvec64;
    marker.Rvec.convertTo(Rvec64, CV_64FC1);
    cv::Rodrigues(Rvec64, rot);
    cv::Mat tran64;
    marker.Tvec.convertTo(tran64, CV_64FC1);

    // Rotate axis direction as to fit ROS (?)
    if (rotate_marker_axis) {
        cv::Mat rotate_to_ros(3, 3, CV_64FC1);
        // -1 0 0
        // 0 0 1
        // 0 1 0
        rotate_to_ros.at<double>(0, 0) = -1.0;
        rotate_to_ros.at<double>(0, 1) = 0.0;
        rotate_to_ros.at<double>(0, 2) = 0.0;
        rotate_to_ros.at<double>(1, 0) = 0.0;
        rotate_to_ros.at<double>(1, 1) = 0.0;
        rotate_to_ros.at<double>(1, 2) = 1.0;
        rotate_to_ros.at<double>(2, 0) = 0.0;
        rotate_to_ros.at<double>(2, 1) = 1.0;
        rotate_to_ros.at<double>(2, 2) = 0.0;
        rot = rot * rotate_to_ros.t();
    }
    tf::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                         rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                         rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

    tf::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

    return tf::Transform(tf_rot, tf_orig);
}

}  // namespace utils

class ArucoMarkerPublisher {
   private:
    // aruco stuff
    aruco::MarkerDetector mDetector_;
    aruco::MarkerDetector mDetector_2;
    aruco::CameraParameters camParam_;
    vector<aruco::Marker> markers_;
    vector<aruco::Marker> markers_big;

    // node params
    bool useRectifiedImages_;
    std::string marker_frame_;
    std::string camera_frame_;
    std::string reference_frame_;
    double marker_size_;
    double marker_size_big;
    bool rotate_marker_axis_;

    // ROS pub-sub
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    image_transport::Publisher image_pub_;
    ros::Publisher marker_pub_;
    tf::TransformListener tfListener_;

    ros::Subscriber cam_info_sub_;
    aruco_msgs::MarkerArray::Ptr marker_msg_;
    cv::Mat inImage_;
    bool useCamInfo_;
    std_msgs::UInt32MultiArray marker_list_msg_;

    tf::StampedTransform lefttoright;

   public:
    ArucoMarkerPublisher()
        : nh_("~"), it_(nh_), useCamInfo_(true) {
        image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);

        nh_.param<bool>("use_camera_info", useCamInfo_, true);
        if (useCamInfo_) {
            sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", nh_);  //, 10.0);
            camParam_ = utils::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);

            nh_.param<double>("marker_size", marker_size_, 0.05);
            nh_.param<double>("marker_size_big", marker_size_big, 0.8);
            nh_.param<bool>("image_is_rectified", useRectifiedImages_, true);
            nh_.param<std::string>("reference_frame", reference_frame_, "");
            nh_.param<std::string>("camera_frame", camera_frame_, "");
            nh_.param<bool>("rotate_marker_axis", rotate_marker_axis_, true);

            // set corner refinement method to LINES
            mDetector_.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
            mDetector_2.setCornerRefinementMethod(aruco::MarkerDetector::LINES);

            lefttoright.setIdentity();
            lefttoright.setOrigin(
                tf::Vector3(
                    -msg->P[3] / msg->P[0],
                    -msg->P[7] / msg->P[5],
                    0.0));

            ROS_ASSERT(not(camera_frame_.empty() and not reference_frame_.empty()));
            if (reference_frame_.empty())
                reference_frame_ = camera_frame_;
        } else {
            camParam_ = aruco::CameraParameters();
        }

        image_pub_ = it_.advertise("result", 1);
        marker_pub_ = nh_.advertise<aruco_msgs::MarkerArray>("aruco_markers", 1);

        marker_msg_ = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
        marker_msg_->header.frame_id = reference_frame_;
        marker_msg_->header.seq = 0;
    }

    bool getTransform(const std::string &refFrame,
                      const std::string &childFrame,
                      tf::StampedTransform &transform) {
        std::string errMsg;

        if (!tfListener_.waitForTransform(refFrame,
                                          childFrame,
                                          ros::Time(0),
                                          ros::Duration(0.5),
                                          ros::Duration(0.01),
                                          &errMsg)) {
            ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
            return false;
        } else {
            try {
                tfListener_.lookupTransform(refFrame, childFrame,
                                            ros::Time(0),  //get latest available
                                            transform);
            } catch (const tf::TransformException &e) {
                ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                return false;
            }
        }
        return true;
    }

    void image_callback(const sensor_msgs::ImageConstPtr &msg) {
        bool publishMarkers = marker_pub_.getNumSubscribers() > 0;
        bool publishImage = image_pub_.getNumSubscribers() > 0;

        // if (!publishMarkers && !publishImage && !publishDebug)
        //     return;

        ros::Time curr_stamp = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            inImage_ = cv_ptr->image;

            //clear out previous detection results
            markers_.clear();

            //Ok, let's detect
            mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
            mDetector_2.detect(inImage_, markers_big, camParam_, marker_size_big, false);

            // marker array publish
            marker_msg_->markers.clear();
            marker_msg_->markers.resize(markers_.size());
            marker_msg_->header.stamp = curr_stamp;
            marker_msg_->header.seq++;

            for (size_t i = 0; i < markers_.size(); ++i) {
                aruco_msgs::Marker &marker_i = marker_msg_->markers.at(i);

                marker_i.header.stamp = curr_stamp;
                marker_i.id = markers_.at(i).id;
                marker_i.confidence = 1.0;
            }

            // if there is camera info let's do 3D stuff
            if (useCamInfo_) {
                //get the current transform from the camera frame to output ref frame
                tf::StampedTransform cameraToReference;
                cameraToReference.setIdentity();

                if (reference_frame_ != camera_frame_) {
                    getTransform(reference_frame_,
                                 camera_frame_,
                                 cameraToReference);
                }

                //Now find the transform for each detected marker
                for (size_t i = 0; i < markers_.size(); ++i) {
                    aruco_msgs::Marker &marker_i = marker_msg_->markers.at(i);

                    aruco::Marker &marker = markers_[i];

                    for (size_t j = 0; j < markers_big.size(); ++j) {
                        if (marker.id == markers_big[j].id) {
                            marker = markers_big[j];
                        }
                    }

                    tf::Transform transform = utils::arucoMarker2Tf(marker, rotate_marker_axis_);

                    // transform object in camera to reference
                    transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(lefttoright) * transform;

                    // create a stamped version with something
                    tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame_, "point_" + std::to_string(marker_i.id));

                    // sendtransform
                    static tf::TransformBroadcaster br;
                    br.sendTransform(stampedTransform);
                    tf::poseTFToMsg(transform, marker_i.pose.pose);
                    marker_i.header.frame_id = reference_frame_;
                }
            }

            //publish marker array
            if (marker_msg_->markers.size() > 0)
                marker_pub_.publish(marker_msg_);

            // Draw detected markers on the image for visualization
            for (size_t i = 0; i < markers_.size(); ++i) {
                markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
            }

            //draw a 3d cube in each marker if there is 3d info
            if (camParam_.isValid() && marker_size_ > 0) {
                for (size_t i = 0; i < markers_.size(); ++i)
                    aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
            }

            // publish input image with markers drawn on it
            if (publishImage) {
                //show input with augmented information
                cv_bridge::CvImage out_msg;
                out_msg.header.stamp = curr_stamp;
                out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                out_msg.image = inImage_;
                image_pub_.publish(out_msg.toImageMsg());
            }

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_marker_publisher");

    ArucoMarkerPublisher node;

    ros::spin();
}
