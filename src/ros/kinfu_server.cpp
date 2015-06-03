/*
 * KinfuServer.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mklingen
 */

#include <ros/kinfu_server.h>

namespace kfusion
{
    
    KinFuServer::KinFuServer(RosRGBDCamera* camera, const std::string& fixedFrame, const std::string& camFrame) :
            should_exit_(false), camera_(camera), baseFrame_(fixedFrame), cameraFrame_(camFrame)
    {
        raycastImgPublisher_ = camera->nodeHandle.advertise<sensor_msgs::Image>("raycast_image", 10);
    }

    void KinFuServer::PublishRaycastImage()
    {
        const int mode = 3;
        kinfu_->renderImage(viewDevice_, mode);
        
        viewHost_.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
        viewDevice_.download(viewHost_.ptr<void>(), viewHost_.step);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = cameraFrame_;
        cv_bridge::CvImage image(header, std::string("rgba8"), viewHost_);
        raycastImgPublisher_.publish(image.toImageMsg());
    }
    
    void KinFuServer::Update()
    {
        bool has_frame = camera_->Grab(lastDepth_, lastColor_);

        if (!has_frame)
        {
            ros::spinOnce();
            return;
        }

        bool has_image = KinFu(lastDepth_, lastColor_);

        if (has_image)
        {
            PublishRaycastImage();
        }

        PublishTransform();
    }

    bool KinFuServer::ExecuteBlocking()
    {
        double time_ms = 0;
        bool has_image = false;

        Initialize();

        kfusion::KinFu& kinfu = *kinfu_;

        ROS_INFO("Starting tracking...\n");
        ros::Rate trackHz(30.0f);
        for (int i = 0; !should_exit_ && ros::ok(); ++i)
        {
            Update();
            ros::spinOnce();
            trackHz.sleep();
        }
        return true;
    }

    bool KinFuServer::KinFu(const cv::Mat& depth, const cv::Mat& color)
    {
        depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
        return(* kinfu_)(depthDevice_);
    }

    bool KinFuServer::ConnectCamera()
    {
        bool cameraConnected = false;

        ROS_INFO("Connecting to camera...\n");
        ros::Rate connectionHz(100.0f);
        ros::Timer printTimer;
        printTimer.setPeriod(ros::Duration(1.0));
        printTimer.start();
        while (!cameraConnected)
        {
            cameraConnected = camera_->hasNewDepth;
            ros::spinOnce();
            connectionHz.sleep();

            if (printTimer.hasPending())
            {
                ROS_INFO("No devices connected yet...\n");
            }
        }
        ROS_INFO("Connected.\n");
        return cameraConnected;
    }

    bool KinFuServer::LoadParams()
    {
        KinFuParams params = KinFuParams::default_params();
        params.cols = camera_->GetDepthWidth();
        params.rows = camera_->GetDepthHeight();
        params.intr = camera_->GetDepthIntrinsics();

        LoadParam(params.bilateral_kernel_size, "bilateral_kernel_size");
        LoadParam(params.bilateral_sigma_depth, "bilateral_sigma_depth");
        LoadParam(params.bilateral_sigma_spatial, "bilateral_sigma_spatial");
        LoadParam(params.gradient_delta_factor, "gradient_delta_factor");
        LoadParam(params.icp_angle_thres, "icp_angle_thresh");
        LoadParam(params.icp_dist_thres, "icp_dist_thresh");
        LoadParam(params.icp_iter_num, "icp_iter_num");
        LoadParam(params.icp_truncate_depth_dist, "icp_truncate_depth_dist");
        LoadParam(params.raycast_step_factor, "raycast_step_factor");
        LoadParam(params.tsdf_max_weight, "tsdf_max_weight");
        LoadParam(params.tsdf_min_camera_movement, "tsdf_min_camera_movement");
        LoadParam(params.tsdf_trunc_dist, "tsdf_trunc_dist");
        LoadParam(params.volume_dims.val[0], "volume_dims_x");
        LoadParam(params.volume_dims.val[1], "volume_dims_y");
        LoadParam(params.volume_dims.val[2], "volume_dims_z");
        LoadParam(params.volume_size.val[0], "volume_size_x");
        LoadParam(params.volume_size.val[1], "volume_size_y");
        LoadParam(params.volume_size.val[2], "volume_size_z");

        float volPosX, volPosY, volPosZ;
        volPosX = params.volume_pose.translation().val[0];
        volPosY = params.volume_pose.translation().val[1];
        volPosZ = params.volume_pose.translation().val[2];

        LoadParam(volPosX, "volume_pos_x");
        LoadParam(volPosY, "volume_pos_y");
        LoadParam(volPosZ, "volume_pos_z");

        params.volume_pose.translate(-params.volume_pose.translation());
        params.volume_pose.translate(cv::Affine3f::Vec3(volPosX, volPosY, volPosZ));

        kinfu_ = KinFu::Ptr(new kfusion::KinFu(params));
        return true;
    }

    bool KinFuServer::PublishTransform()
    {
        tf::StampedTransform currTf;
        currTf.child_frame_id_ = cameraFrame_;
        currTf.frame_id_ = baseFrame_;
        currTf.stamp_ = ros::Time::now();
        cv::Affine3f currPose = kinfu_->getCameraPose();
        currTf.setOrigin(tf::Vector3(currPose.translation().val[0], currPose.translation().val[1], currPose.translation().val[2]));
        cv::Affine3f::Mat3 rot = currPose.rotation();
        tf::Matrix3x3 tfRot(rot.val[0], rot.val[1], rot.val[2], rot.val[3], rot.val[4], rot.val[5], rot.val[6], rot.val[7], rot.val[8]);
        tf::Quaternion tfQuat;
        tfRot.getRotation(tfQuat);
        currTf.setRotation(tfQuat);
        tfBroadcaster_.sendTransform(currTf);
        return true;
    }

} /* namespace kfusion */
