#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <kfusion/kinfu.hpp>
#include <ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace kfusion;

struct KinFuNode
{

     KinFuNode(RosRGBDCamera* camera, const std::string& fixedFrame, const std::string& camFrame) : exit_ (false),  iteractive_mode_(false), camera_(camera), baseFrame(fixedFrame), cameraFrame(camFrame)
    {
             raycastImgPublisher = camera->nodeHandle.advertise<sensor_msgs::Image>("raycast_image", 10);
    }

     void show_raycasted(KinFu& kinfu)
        {
           const int mode = 3;
           if (iteractive_mode_)
               kinfu.renderImage(view_device_, kinfu.getCameraPose(), mode);
           else
               kinfu.renderImage(view_device_, mode);

           view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
           view_device_.download(view_host_.ptr<void>(), view_host_.step);
           std_msgs::Header header;
           header.stamp = ros::Time::now();
           header.frame_id = cameraFrame;
           cv_bridge::CvImage image(header, std::string("rgba8"), view_host_);
           raycastImgPublisher.publish(image.toImageMsg());
        }


    bool execute()
    {
        cv::Mat depth, image;
        double time_ms = 0;
        bool has_image = false;
        bool cameraConnected = false;

        ROS_INFO("Connecting to camera...\n");
        ros::Rate connectionHz(100.0f);
        while (!cameraConnected)
        {
                cameraConnected = camera_->hasNewDepth;
                ros::spinOnce();
                connectionHz.sleep();
        }

        KinFuParams params = KinFuParams::default_params();
        params.cols = camera_->GetDepthWidth();
        params.rows = camera_->GetDepthHeight();
        params.intr = camera_->GetDepthIntrinsics();
        kinfu_ = KinFu::Ptr( new KinFu(params) );
        KinFu& kinfu = *kinfu_;


        ROS_INFO("Starting tracking...\n");
        ros::Rate trackHz(30.0f);
        for (int i = 0; !exit_ && ros::ok(); ++i)
        {
            bool has_frame = camera_->Grab(depth, image);

            if (!has_frame)
            {
                    ros::spinOnce();
                    continue;
            }
            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);

            {
                SampledScopeTime fps(time_ms); (void)fps;
                has_image = kinfu(depth_device_);
            }

            if (has_image)
            {
                    show_raycasted(kinfu);
            }

            tf::StampedTransform currTf;
            currTf.child_frame_id_ = cameraFrame;
            currTf.frame_id_ = baseFrame;
            currTf.stamp_ = ros::Time::now();
            cv::Affine3f currPose = kinfu.getCameraPose();
            currTf.setOrigin(tf::Vector3(currPose.translation().val[0], currPose.translation().val[1], currPose.translation().val[2]));
            cv::Affine3f::Mat3 rot = currPose.rotation();
            tf::Matrix3x3 tfRot(rot.val[0], rot.val[1], rot.val[2],
                            rot.val[3], rot.val[4], rot.val[5],
                            rot.val[6], rot.val[7], rot.val[8]);
            tf::Quaternion tfQuat;
            tfRot.getRotation(tfQuat);
            currTf.setRotation(tfQuat);
            tfBroadcaster.sendTransform(currTf);
            ros::spinOnce();
            trackHz.sleep();
        }
        return true;
    }

    bool exit_, iteractive_mode_;
    KinFu::Ptr kinfu_;
    RosRGBDCamera* camera_;
    cv::Mat view_host_;
    cuda::Image view_device_;
    cuda::Depth depth_device_;
    ros::Publisher raycastImgPublisher;
    std::string baseFrame;
    std::string cameraFrame;
    tf::TransformBroadcaster tfBroadcaster;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[])
{
    int device = 0;
    cuda::setDevice (device);
    cuda::printShortCudaDeviceInfo (device);

    if(cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    ros::init(argc, argv, "kinfu_ros");

    ros::NodeHandle node("~");
    RosRGBDCamera camera(node);
    camera.SubscribeDepth("/camera/depth/image_raw");
    camera.SubscribeRGB("/camera/rgb/image_rect_color");
    KinFuNode app (&camera, "/map", "/camera_depth_optical_frame");
    app.execute ();

    return 0;
}
