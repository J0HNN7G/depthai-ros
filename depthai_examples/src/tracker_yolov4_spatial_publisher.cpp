#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackSpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

const std::vector<std::string> label_map = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

dai::Pipeline createPipeline(bool syncNN, bool subpixel, std::string nnPath, int confidence, int LRchecktresh, std::string monoResolutionStr, bool fullFrameTracking) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto tracker = pipeline.create<dai::node::ObjectTracker>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutDepth->setStreamName("depth");
    xoutTracker->setStreamName("tracklets");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    if(monoResolutionStr == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
    } else if(monoResolutionStr == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    } else if(monoResolutionStr == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
    } else if(monoResolutionStr == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolutionStr.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    /// setting node configs
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.3f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // object tracker settings
    tracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    tracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    tracker->passthroughTrackerFrame.link(xoutRgb->input);

    if(fullFrameTracking) {
        colorCam->setPreviewKeepAspectRatio(false);
        colorCam->video.link(tracker->inputTrackerFrame);
        tracker->inputTrackerFrame.setBlocking(false);
        //  do not block the pipeline if it's too slow on full frame
        tracker->inputTrackerFrame.setQueueSize(2);
    } else {
        spatialDetectionNetwork->passthrough.link(tracker->inputTrackerFrame);
    }

    spatialDetectionNetwork->passthrough.link(tracker->inputDetectionFrame);
    spatialDetectionNetwork->out.link(tracker->inputDetections);
    tracker->out.link(xoutTracker->input);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tracker_yolov4_spatial_node");

    std::string nSpace, tfPrefix, resourceBaseFolder, nnPath;
    std::string camera_param_uri;
    std::string nnName(BLOB_NAME);  // Set your blob name for the model here
    bool syncNN, subpixel, fullFrameTracking;
    int confidence = 200, LRchecktresh = 5;
    std::string monoResolution = "400p";

    node->declare_parameter("namespace", "");
    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("confidence", confidence);
    node->declare_parameter("LRchecktresh", LRchecktresh);
    node->declare_parameter("monoResolution", monoResolution);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("fullFrameTracking", false);

    node->get_parameter("namespace", nSpace);
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("fullFrameTracking", fullFrameTracking);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    dai::Pipeline pipeline = createPipeline(syncNN, subpixel, nnPath, confidence, LRchecktresh, monoResolution, fullFrameTracking);
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto trackQueue = device.getOutputQueue("tracklets", 30, false);
    auto calibrationHandler = device.readCalibration();

    int colorWidth, colorHeight;
    if (!fullFrameTracking) {
        colorWidth = 416;
        colorHeight = 416;
    } else {
        colorWidth = 1920;
        colorHeight = 1080;
    }

    int monoWidth, monoHeight;
    if(monoResolution == "720p") {
        monoWidth = 1280;
        monoHeight = 720;
    } else if(monoResolution == "400p") {
        monoWidth = 640;
        monoHeight = 400;
    } else if(monoResolution == "800p") {
        monoWidth = 1280;
        monoHeight = 800;
    } else if(monoResolution == "480p") {
        monoWidth = 640;
        monoHeight = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }


    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter rgbConverter(nSpace + '/' + tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, colorWidth, colorHeight);
    std::string rgbTopicName = tfPrefix + "/rgb/image_raw";
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
                                                                                       node,
                                                                                       rgbTopicName,
                                                                                       std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                 &rgbConverter,  // since the converter has the same frame name
                                                                                                                 // and image type is also same we can reuse it
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2),
                                                                                       30,
                                                                                       rgbCameraInfo,
                                                                                       tfPrefix + "/rgb");

    dai::rosBridge::TrackSpatialDetectionConverter trackConverter(nSpace + '/' + tfPrefix + "_rgb_camera_optical_frame", 416, 416, fullFrameTracking, 0.3);

    std::string trackTopicName = tfPrefix + "/nn/yolov4_Spatial_tracklets";
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::TrackDetection2DArray, dai::Tracklets> trackPublish(
        trackQueue,
        node,
        trackTopicName,
        std::bind(&dai::rosBridge::TrackSpatialDetectionConverter::toRosMsg, &trackConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    dai::rosBridge::ImageConverter depthConverter(nSpace + '/' + tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    std::string depthTopicName = tfPrefix + "/stereo/image_raw";
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        depthQueue,
        node,
        depthTopicName,
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        tfPrefix + "/stereo");

    depthPublish.addPublisherCallback();
    trackPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();

    rclcpp::spin(node);

    return 0;
}
