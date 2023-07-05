#include "seek_thermal/seek.h"
#include "seek_thermal/SeekLogging.h"
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <camera_info_manager/camera_info_manager.hpp>





class SeekThermal_Camera : public rclcpp::Node
{
private:
    std::string nodeName;

    image_transport::CameraPublisher pub;
    image_transport::Publisher thermal_image_pub;
    //ROS1 CAM_INFO
    std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager;
    std::unique_ptr<LibSeek::SeekCam> seek_cam;
    std::string frame_id;
    std::string model;
    cv::Mat ffc_image;

    /**
     * Calibration parameters:
     *  - cal_beta: Temperature sensitivity
     *  - linear_k: Linear model slope for device temperature compensation
     *  - linear_offset: Linear model offset for device temperature compensation
     */
    float cal_beta;
    float linear_k;
    float linear_offset;


    //-------------------------------------------------------------------------------- TODO : RE ADD SEEKROSCALLBACK & LOG
    
public:
    SeekThermal_Camera(): Node("SeekThermal_Camera")
    { 
        RCLCPP_INFO(this->get_logger(), "seek");
        model = this->declare_parameter<std::string>("type", "seekpro");
        std::string cameraName = this->declare_parameter<std::string>("camera_name", "seekpro");
        std::string cameraInfoUrl = this->declare_parameter<std::string>("camera_info_url", "");
        frame_id = this->declare_parameter<std::string>("frame_id", "seekpro_optical");   
        std::string ffc_filename = this->declare_parameter<std::string>("ffc_image", "");
        
        if (ffc_filename != "")
        {   
            RCLCPP_INFO(this->get_logger(),"Loading flat field calibration from ", ffc_filename);
            ffc_image = cv::imread(ffc_filename, cv::ImreadModes::IMREAD_UNCHANGED);
            if (ffc_image.empty())
            {
                RCLCPP_INFO(this->get_logger(),"Failed to load flat field calibration!");
            }
        }
        cal_beta = this->declare_parameter<float>("cal_beta", 200.0f);
        linear_k = this->declare_parameter<float>("linear_k", -1.5276f);
        linear_offset = this->declare_parameter<float>("linear_offset", -470.8979f);

        
        
        //ROS1 CAM_INFO
        info_manager.reset(new camera_info_manager::CameraInfoManager(this, cameraName, cameraInfoUrl));



        rclcpp::NodeOptions options;
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
        image_transport::ImageTransport it(node);

        pub = it.advertiseCamera("image_raw", 1);
        thermal_image_pub = it.advertise("thermal_image", 1);
    }


    ~SeekThermal_Camera() {
        RCLCPP_INFO(this->get_logger(),"DESTRUCTION");
    }

    


    void reset()
    {
        RCLCPP_INFO(this->get_logger(),"reset");
        if (model == "seekpro")
        {   
            RCLCPP_INFO(this->get_logger(), "modelseekpro");
            seek_cam.reset(new LibSeek::SeekThermalPro(ffc_image));
        }
        else if (model == "seek")
        {   
            RCLCPP_INFO(this->get_logger(), "modelseek");
            seek_cam.reset(new LibSeek::SeekThermal(ffc_image));
        }
        else
        {
            
            RCLCPP_INFO(this->get_logger(), "Unknown model type: ", model);
            rclcpp::shutdown();
        }
    }

    bool init(int num_retries)
    {
        RCLCPP_INFO(this->get_logger(),"init");
        while (num_retries-- != 0)
        {
            reset();
            if (seek_cam->open())
            {
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "Could not initialize camera, retrying");
            rclcpp::sleep_for(std::chrono::milliseconds(500));    
        }
        return false;
    }

    void getThermalImage(const cv::Mat& src, cv::Mat& dst)
    {
        dst.create(src.size(), CV_32FC1);
        
        auto device_k = [](float beta, int sensor_value) -> float {
            constexpr auto ref_temp = 297.0f;
            constexpr auto ref_sensor = 6616.0f;
            float part3 = std::log(sensor_value/ref_sensor);
            float parte = part3 / beta + 1.0 / ref_temp;
            return 1.0 / parte;
        }(cal_beta, seek_cam->device_temp_sensor());

        auto temp_k = [&](uint16_t raw_value) -> float {
            const float raw_scaled = raw_value * 330 / 16384.0f;
            return raw_scaled - device_k * linear_k + linear_offset - 273.0f;
        };

        for(auto row = 0; row < src.rows; ++row)
        {
            for(auto col = 0; col < src.cols; ++col)
            {
                dst.at<float>(row, col) = temp_k(src.at<uint16_t>(row, col));
            }
        }
    }

    bool publish()
    {
        RCLCPP_INFO(this->get_logger(), "publish");
        if (!seek_cam || !seek_cam->isOpened()){
            return false;
        } 
        
        if (!info_manager){
            return false;
        }

        cv::Mat thermalImg;
        if(!seek_cam->read(thermalImg))
        {   
            RCLCPP_INFO(this->get_logger(), "Failed to read thermal image");
            rclcpp::sleep_for(std::chrono::milliseconds(100));  
            return false;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "frame_id";

        if (pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImagePtr pubImg = std::make_shared<cv_bridge::CvImage>(header, "16UC1", thermalImg);
            sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo = std::make_shared<sensor_msgs::msg::CameraInfo>(info_manager->getCameraInfo());
            cameraInfo->header.stamp = header.stamp;
            cameraInfo->header.frame_id = header.frame_id;
            pub.publish(pubImg->toImageMsg(), cameraInfo);
        }
        
        if (thermal_image_pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImagePtr thermalImgProcessed = std::make_shared<cv_bridge::CvImage>(header, "32FC1");
            getThermalImage(thermalImg, thermalImgProcessed->image);
            thermal_image_pub.publish(thermalImgProcessed->toImageMsg());
        }
        return true;
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto seek = std::make_shared<SeekThermal_Camera>();
    RCLCPP_INFO(seek->get_logger(), "mainHello");
    while(rclcpp::ok()){
        if (!seek->publish())
        {
            seek->init(10); //TODO change this value
        }
        rclcpp::spin_some(seek);
    }
    rclcpp::shutdown();
    return 0;
}