#include "hik_camera/hik_camera.hpp"

namespace camera
{

CameraNode::CameraNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    RCLCPP_INFO(this -> get_logger(), "Creating");
    
    // 声明ros2中参数
    this -> declare_parameter<int>("width", 1280);
    this -> declare_parameter<int>("height", 1024);
    this -> declare_parameter<bool>("FrameRateEnable", false);
    this -> declare_parameter<int>("FrameRate", 10);
    this -> declare_parameter<int>("BurstFrameCount", 10);
    this -> declare_parameter<int>("ExposureTime", 25000);
    this -> declare_parameter<bool>("GammaEnable", true);
    this -> declare_parameter<double>("Gamma", 0.55);
    this -> declare_parameter<int>("GainAuto", 2);
    this -> declare_parameter<bool>("SaturationEnable", true);
    this -> declare_parameter<int>("Saturation", 128);
    this -> declare_parameter<int>("Offset_x", 0);
    this -> declare_parameter<int>("Offset_y", 0);
    this -> declare_parameter<int>("TriggerMode", 1);
    this -> declare_parameter<int>("TriggerSource", 2);
    this -> declare_parameter<int>("LineSelector", 2);

    // 为成员变量赋值，设置yaml文件中的值，否则为默认参数
    this -> get_parameter("width", width);
    this -> get_parameter("height", height);
    this -> get_parameter("FrameRateEnable", FrameRateEnable);
    this -> get_parameter("FrameRate", FrameRate);
    this -> get_parameter("BurstFrameCount", BurstFrameCount);
    this -> get_parameter("ExposureTime", ExposureTime);
    this -> get_parameter("GammaEnable", GammaEnable);
    this -> get_parameter("Gamma", Gamma);
    this -> get_parameter("GainAuto", GainAuto);
    this -> get_parameter("SaturationEnable", SaturationEnable);
    this -> get_parameter("Saturation", Saturation);
    this -> get_parameter("Offset_x", Offset_x);
    this -> get_parameter("Offset_y", Offset_y);
    this -> get_parameter("TriggerMode", TriggerMode);
    this -> get_parameter("TriggerSource", TriggerSource);
    this -> get_parameter("LineSelector", LineSelector);
    

    handle = NULL;
    // 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    if (stDeviceList.nDeviceNum > 0) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo) {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else {
        printf("Find No Devices!\n");
        exit(-1);
    }

    // 选择设备并创建句柄 
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    // 设置相机参数，读取的是成员变量
    printf("set configure in yaml!\n");
    this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
    if (FrameRateEnable)
        this->set(CAP_PROP_FRAMERATE, FrameRate);
    this->set(CAP_PROP_HEIGHT, height);
    this->set(CAP_PROP_WIDTH, width);
    this->set(CAP_PROP_OFFSETX, Offset_x);
    this->set(CAP_PROP_OFFSETY, Offset_y);
    this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
    this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
    if (GammaEnable)
        this->set(CAP_PROP_GAMMA, Gamma);
    this->set(CAP_PROP_GAINAUTO, GainAuto);

    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
    if (MV_OK == nRet) {
        printf("set BalanceRatio OK! value=%f\n",0.0 );
    }
    else {
        printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
    }
    this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
    if (SaturationEnable)
        this->set(CAP_PROP_SATURATION, Saturation);
    

    //软件触发
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK == nRet) {
        printf("set TriggerMode OK!\n");
    }
    else {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    }

    //********** 图像格式 **********/
    // 0x01100003:Mono10
    // 0x010C0004:Mono10Packed
    // 0x01100005:Mono12
    // 0x010C0006:Mono12Packed
    // 0x01100007:Mono16
    // 0x02180014:RGB8Packed
    // 0x02100032:YUV422_8
    // 0x0210001F:YUV422_8_UYVY
    // 0x01080008:BayerGR8
    // 0x01080009:BayerRG8
    // 0x0108000A:BayerGB8
    // 0x0108000B:BayerBG8
    // 0x0110000e:BayerGB10
    // 0x01100012:BayerGB12
    // 0x010C002C:BayerGB12Packed

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014); // 目前 RGB  
    if (MV_OK == nRet) {
        printf("set PixelFormat OK ! value = RGB\n");
    }
    else {
        printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
    }
    MVCC_ENUMVALUE t = {0};
    

    nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
    if (MV_OK == nRet) {
        printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
    }
    else {
        printf("get PixelFormat fail! nRet [%x]\n", nRet);
    }

    //********** 开始取流**********/
    nRet = MV_CC_StartGrabbing(handle);

    if (MV_OK != nRet) {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    this -> pub_ = image_transport::create_camera_publisher(this, "~/image_raw");
    this -> cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    auto camera_calibration_file_param_ = this -> declare_parameter("camera_calibration_file",
         "file://config/camera_calibration.yaml");
    cinfo_manager_ -> loadCameraInfo(camera_calibration_file_param_);
    
    std::unique_lock<std::shared_mutex> thread_lock(worker_thread_mutex);
    worker_thread = std::thread([this](){
        this -> HKWorkThread(this -> handle);
    });
    
}

CameraNode::CameraNode(const rclcpp::NodeOptions& options)
    : CameraNode::CameraNode("hik_camera", options)
    {}


CameraNode::~CameraNode() {
    int nRet;
    worker_thread.join();

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_StopGrabbing succeed.\n");
    // 关闭设备
    //********** frame **********/

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_CloseDevice succeed.\n");
    // 销毁句柄
    //********** frame **********/

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_DestroyHandle succeed.\n");
}



bool CameraNode::set(CamerProperties type, float value) {
    switch (type) {
        case CAP_PROP_FRAMERATE_ENABLE :
            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
            if (MV_OK == nRet) 
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            else 
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_FRAMERATE :
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
            if (MV_OK == nRet)
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            else
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_BURSTFRAMECOUNT :
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (MV_OK == nRet)
                printf("set AcquisitionBurstFrameCount OK!\n");
            else
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_HEIGHT :
            nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度
            if (MV_OK == nRet)
                printf("set Height OK!\n");
            else
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_WIDTH :
            nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度
            if (MV_OK == nRet)
                printf("set Width OK!\n");
            else
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_OFFSETX :
            nRet = MV_CC_SetIntValue(handle, "OffsetX", value); //图像宽度
            if (MV_OK == nRet)
                printf("set Offset X OK!\n");
            else
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_OFFSETY :
            nRet = MV_CC_SetIntValue(handle, "OffsetY", value); //图像宽度
            if (MV_OK == nRet)
                printf("set Offset Y OK!\n");
            else
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_EXPOSURE_TIME :
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间
            if (MV_OK == nRet)
                printf("set ExposureTime OK! value=%f\n",value);
            else
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_GAMMA_ENABLE :
            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）
            if (MV_OK == nRet)
                printf("set GammaEnable OK! value=%f\n",value);
            else
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_GAMMA:
            nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大
            if (MV_OK == nRet)
                printf("set Gamma OK! value=%f\n",value);
            else
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_GAINAUTO:
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //亮度 越大越亮
            if (MV_OK == nRet)
                printf("set GainAuto OK! value=%f\n",value);
            else
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_SATURATION_ENABLE :
            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)
            if (MV_OK == nRet)
                printf("set SaturationEnable OK! value=%f\n",value);
            else
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_SATURATION :
            nRet = MV_CC_SetIntValue(handle, "Saturation", value); //饱和度 默认128 最大255
            if (MV_OK == nRet)
                printf("set Saturation OK! value=%f\n",value);
            else
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_TRIGGER_MODE :
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); //饱和度 默认128 最大255
            if (MV_OK == nRet)
                printf("set TriggerMode OK!\n");
            else
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_TRIGGER_SOURCE:
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value); //饱和度 默认128 最大255255
            if (MV_OK == nRet)
                printf("set TriggerSource OK!\n");
            else
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            break;
        case CAP_PROP_LINE_SELECTOR:
            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); //饱和度 默认128 最大255
            if (MV_OK == nRet)
                printf("set LineSelector OK!\n");
            else
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            break;
        default :
            return 0;
    }
    return nRet;
}

bool CameraNode::reset() {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
}


bool CameraNode::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
    if (NULL == pstMVDevInfo){
        printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo -> nTLayerType == MV_GIGE_DEVICE){
        printf("%s %x\n", "nCurrentIp:", pstMVDevInfo -> SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
        printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo -> SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
    }
    else if (pstMVDevInfo -> nTLayerType == MV_USB_DEVICE) {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo -> SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else {
        printf("Not support.\n");
    }
    return true;
}


//^ ********************************** HKWorkThread1 ************************************ //
void* CameraNode::HKWorkThread(void* p_handle) {
    RCLCPP_INFO(get_logger(), "In HKWorkTHread");
    rclcpp::Rate loop_rate(20);
    
    int nRet;
    cv::Mat frame;    
    double start;
    size_t frame_id = 0;
    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
            cinfo_manager_ -> getCameraInfo());
    
    
    unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
    unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    
    RCLCPP_INFO(get_logger(), "start publishing image msgs");
    int image_empty_count = 0; //空图帧数
    while (rclcpp::ok()) {
        start = static_cast<double>(cv::getTickCount());
        auto msg = std::make_unique<sensor_msgs::msg::Image>();

        nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
        if (nRet != MV_OK) {
            if (++image_empty_count > 100) {
                RCLCPP_INFO(get_logger(), "The Number of Faild Reading Exceed The Set Value!\n");
                exit(-1);
            }
            continue;
        }

        image_empty_count = 0; //空图帧数
        // 转换图像格式为BGR8
        stConvertParam.nWidth = 1280;                               //ch:图像宽 | en:image width
        stConvertParam.nHeight = 1024;                              //ch:图像高 | en:image height
        stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format //! 输出格式 RGB
        stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
        stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format //! 输入格式 RGB
        MV_CC_ConvertPixelType(p_handle, &stConvertParam);
        
        
        frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();
        convert_frame_to_message(frame, frame_id, *msg, *camera_info_msg);
        pub_.publish(std::move(msg), camera_info_msg);
        ++frame_id;
        

        //double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
        loop_rate.sleep();
    }
    free(m_pBufForDriver);
    free(m_pBufForSaveImage);
    return 0;
}


std::string CameraNode::mat_type2encoding(int mat_type) {
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

void CameraNode::convert_frame_to_message(const cv::Mat & frame, size_t frame_id,
    sensor_msgs::msg::Image & msg, 
    sensor_msgs::msg::CameraInfo & camera_info_msg) {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this->get_clock()->now();

    msg.header.frame_id = std::to_string(frame_id);
    msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = std::to_string(frame_id);
    camera_info_msg.header.stamp = timestamp;
}


} // namespace camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)
