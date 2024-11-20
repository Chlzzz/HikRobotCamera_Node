#ifndef HIK_CAMERA_H_
#define HIK_CAMERA_H

#include <stdio.h>
#include <thread>
#include <opencv2/opencv.hpp>

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>


namespace camera
{

#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)

enum CamerProperties
{
    CAP_PROP_FRAMERATE_ENABLE,  // 帧数可调
    CAP_PROP_FRAMERATE,         // 帧数
    CAP_PROP_BURSTFRAMECOUNT,   // 外部一次触发帧数
    CAP_PROP_HEIGHT,            // 图像高度
    CAP_PROP_WIDTH,             // 图像宽度
    CAP_PROP_EXPOSURE_TIME,     // 曝光时间
    CAP_PROP_GAMMA_ENABLE,      // 伽马因子可调
    CAP_PROP_GAMMA,             // 伽马因子
    CAP_PROP_GAINAUTO,          // 亮度
    CAP_PROP_SATURATION_ENABLE, // 饱和度可调
    CAP_PROP_SATURATION,        // 饱和度
    CAP_PROP_OFFSETX,           // X偏置
    CAP_PROP_OFFSETY,           // Y偏置
    CAP_PROP_TRIGGER_MODE,      // 外部触发
    CAP_PROP_TRIGGER_SOURCE,    // 触发源
    CAP_PROP_LINE_SELECTOR      // 触发线
};

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const std::string& node_name, 
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit CameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CameraNode();

public:
    void* HKWorkThread(void *p_handle);                         // 任务， 发布image
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);      // 输出摄像头信息
    bool set(camera::CamerProperties type, float value);        // 设置摄像头参数
    bool reset();                                               // 恢复默认参数

private:
    void* handle;     
    std::thread worker_thread;                                  // 工作线程
    std::shared_mutex worker_thread_mutex; 

    image_transport::CameraPublisher pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

    void convert_frame_to_message(const cv::Mat & frame, size_t frame_id,
        sensor_msgs::msg::Image & msg, sensor_msgs::msg::CameraInfo & camera_info_msg);
    std::string mat_type2encoding(int mat_type);

private:
    // yaml config 
    int nRet;
    int width;
    int height;
    int Offset_x;
    int Offset_y;
    bool FrameRateEnable;
    int FrameRate;
    int BurstFrameCount;
    int ExposureTime;
    bool GammaEnable;
    float Gamma;
    int GainAuto;
    bool SaturationEnable;
    int Saturation;
    int TriggerMode;
    int TriggerSource;
    int LineSelector;
};


} // namespace camera

#endif // HIK_CAMERA_H_
