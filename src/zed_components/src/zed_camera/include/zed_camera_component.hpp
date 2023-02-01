// Copyright 2022 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZED_CAMERA_COMPONENT_HPP_
#define ZED_CAMERA_COMPONENT_HPP_

#include <rcutils/logging_macros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sl/Camera.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <zed_interfaces/msg/depth_info_stamped.hpp>
#include <zed_interfaces/msg/object.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <zed_interfaces/msg/plane_stamped.hpp>
#include <zed_interfaces/srv/set_pose.hpp>
#include <zed_interfaces/srv/set_roi.hpp>
#include <zed_interfaces/srv/start_svo_rec.hpp>

#include "sl_tools.hpp"
#include "visibility_control.hpp"

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0, 0, RCL_SYSTEM_TIME)

namespace stereolabs
{
// ----> Typedefs to simplify declarations

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub;
typedef std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> disparityPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::MagneticField>> magPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::FluidPressure>> pressPub;
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>> tempPub;

typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posePub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>>
  poseCovPub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TransformStamped>> transfPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub;
typedef std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pathPub;

typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::ObjectsStamped>> objPub;
typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::DepthInfoStamped>> depthInfoPub;

typedef std::shared_ptr<rclcpp::Publisher<zed_interfaces::msg::PlaneStamped>> planePub;
typedef std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPub;

typedef std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PointStamped>> clickedPtSub;

typedef std::unique_ptr<sensor_msgs::msg::Image> imageMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::Imu> imuMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::FluidPressure> pressMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::Temperature> tempMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::MagneticField> magMsgPtr;
typedef std::unique_ptr<stereo_msgs::msg::DisparityImage> dispMsgPtr;

typedef std::unique_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseCovMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::TransformStamped> transfMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Odometry> odomMsgPtr;
typedef std::unique_ptr<nav_msgs::msg::Path> pathMsgPtr;

typedef std::unique_ptr<zed_interfaces::msg::ObjectsStamped> objDetMsgPtr;
typedef std::unique_ptr<zed_interfaces::msg::DepthInfoStamped> depthInfoMsgPtr;
typedef std::unique_ptr<zed_interfaces::msg::PlaneStamped> planeMsgPtr;
typedef std::unique_ptr<visualization_msgs::msg::Marker> markerMsgPtr;

typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetOdomSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetPosTrkSrvPtr;
typedef rclcpp::Service<zed_interfaces::srv::SetPose>::SharedPtr setPoseSrvPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableObjDetPtr;
typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableMappingPtr;
typedef rclcpp::Service<zed_interfaces::srv::StartSvoRec>::SharedPtr startSvoRecSrvPtr;
typedef rclcpp::Service<zed_interfaces::srv::SetROI>::SharedPtr setRoiSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopSvoRecSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pauseSvoSrvPtr;
typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetRoiSrvPtr;

/*!
   * @brief Video/Depth topic resolution
   */
typedef enum
{
  HD2K,  //!< 2208x1242
  HD1080,  //!< 1920x1080
  HD720,  //!< 1280x720
  MEDIUM,  //!< 896x512
  VGA,  //!< 672x376
  LOW  //!< Half-MEDIUM 448x256
} PubRes;
// <---- Typedefs to simplify declarations

class ZedCamera : public rclcpp::Node
{
public:
  ZED_COMPONENTS_PUBLIC
  explicit ZedCamera(const rclcpp::NodeOptions & options);

  virtual ~ZedCamera();

protected:
  // ----> Initialization functions
  void initParameters();
  void initServices();

  void getDebugParams();
  void getGeneralParams();
  void getVideoParams();
  void getDepthParams();
  void getPosTrackingParams();
  void getSensorsParams();
  void getMappingParams();
  void getOdParams();

  void setTFCoordFrameNames();
  void initPublishers();
  void fillCamInfo(
    sl::Camera & zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
    std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, std::string leftFrameId,
    std::string rightFrameId, bool rawParam = false);

  bool startCamera();
  bool startPosTracking();
  bool start3dMapping();
  void stop3dMapping();
  bool startObjDetect();
  void stopObjDetect();
  bool startSvoRecording(std::string & errMsg);
  void stopSvoRecording();
  // <---- Initialization functions

  // ----> Callbacks
  void threadFunc_pubVideoDepth();
  void callback_pubFusedPc();
  void callback_pubPaths();
  void callback_pubTemp();
  rcl_interfaces::msg::SetParametersResult callback_paramChange(
    std::vector<rclcpp::Parameter> parameters);
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void callback_resetOdometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_resetPosTracking(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_setPose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::SetPose_Request> req,
    std::shared_ptr<zed_interfaces::srv::SetPose_Response> res);
  void callback_enableObjDet(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
  void callback_enableMapping(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool_Request> req,
    std::shared_ptr<std_srvs::srv::SetBool_Response> res);
  void callback_startSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request> req,
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response> res);
  void callback_stopSvoRec(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_pauseSvoInput(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  void callback_clickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void callback_setRoi(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zed_interfaces::srv::SetROI_Request> req,
    std::shared_ptr<zed_interfaces::srv::SetROI_Response> res);
  void callback_resetRoi(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> res);
  // <---- Callbacks

  // ----> Thread functions
  void threadFunc_zedGrab();
  void threadFunc_pointcloudElab();
  void threadFunc_pubSensorsData();
  // <---- Thread functions

  // ----> Publishing functions
  void publishImageWithInfo(
    sl::Mat & img, image_transport::CameraPublisher & pubImg, camInfoMsgPtr & camInfoMsg,
    std::string imgFrameId, rclcpp::Time t);
  void publishDepthMapWithInfo(sl::Mat & depth, rclcpp::Time t);
  void publishDisparity(sl::Mat disparity, rclcpp::Time t);

  bool areVideoDepthSubscribed();
  void retrieveVideoDepth();
  void publishVideoDepth(rclcpp::Time & out_pub_ts);
  void publishPointCloud();
  void publishImuFrameAndTopic();

  void publishOdom(tf2::Transform & odom2baseTransf, sl::Pose & slPose, rclcpp::Time t);
  void publishPose();
  void publishTFs(rclcpp::Time t);
  void publishOdomTF(rclcpp::Time t);
  void publishPoseTF(rclcpp::Time t);
  rclcpp::Time publishSensorsData(rclcpp::Time t = TIMEZERO_ROS);
  // <---- Publishing functions

  // ----> Utility functions
  bool isDepthRequired();
  bool isPosTrackingRequired();

  void applyVideoSettings();
  void applyDepthSettings();

  void processOdometry();
  void processPose();

  void processDetectedObjects(rclcpp::Time t);

  bool setPose(float xt, float yt, float zt, float rr, float pr, float yr);
  void initTransforms();
  bool getSens2BaseTransform();
  bool getSens2CameraTransform();
  bool getCamera2BaseTransform();

  void startFusedPcTimer(double fusedPcRate);
  void startPathPubTimer(double pathTimerRate);
  void startTempPubTimer();

  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal, std::string log_info = std::string(),
    bool dynamic = false);

  // Region of Interest
  std::string getParam(std::string paramName, std::vector<std::vector<float>> & outVal);
  std::string parseRoiPoly(
    const std::vector<std::vector<float>> & in_poly, std::vector<sl::float2> & out_poly);
  void resetRoi();
  // <---- Utility functions

private:
  // ZED SDK
  sl::Camera mZed;
  sl::InitParameters mInitParams;
  sl::RuntimeParameters mRunParams;

  uint64_t mFrameCount = 0;

  // ----> Topics
  std::string mTopicRoot = "~/";
  std::string mOdomTopic;
  std::string mPoseTopic;
  std::string mPoseCovTopic;
  std::string mPointcloudFusedTopic;
  std::string mObjectDetTopic;
  std::string mOdomPathTopic;
  std::string mMapPathTopic;
  std::string mClickedPtTopic;  // Clicked point
  // <---- Topics

  // ----> Parameter variables
  bool mDebugMode = false;
  bool mDebugSensors = false;
  int mCamId = 0;
  int mCamSerialNumber = 0;
  sl::MODEL mCamUserModel = sl::MODEL::ZED;  // Default camera model
  sl::MODEL mCamRealModel;                   // Camera model requested to SDK
  unsigned int mCamFwVersion;                // Camera FW version
  unsigned int mSensFwVersion;               // Sensors FW version
  std::string mCameraName = "zed";           // Default camera name
  int mCamGrabFrameRate = 15;
  std::string mSvoFilepath = "";
  bool mSvoLoop = false;
  bool mSvoRealtime = false;
  int mVerbose = 1;
  int mGpuId = -1;
  sl::RESOLUTION mCamResol = sl::RESOLUTION::HD720;  // Default resolution: RESOLUTION_HD720
  PubRes mPubResolution = MEDIUM;  // Use native DNN resolution for NEURAL depth to improve speed and quality.
  sl::DEPTH_MODE mDepthQuality =
    sl::DEPTH_MODE::PERFORMANCE;  // Default depth mode: DEPTH_MODE_PERFORMANCE
  bool mDepthDisabled =
    false;  // Indicates if depth calculation is not required (DEPTH_MODE::NONE se for )
  bool mDepthStabilization = true;
  std::vector<std::vector<float>> mRoiParam;
  int mCamTimeoutSec = 5;
  int mMaxReconnectTemp = 5;
  bool mCameraSelfCalib = true;
  bool mCameraFlip = false;
  sl::SENSING_MODE mDepthSensingMode =
    sl::SENSING_MODE::STANDARD;   // Default Sensing mode: SENSING_MODE_STANDARD
  bool mOpenniDepthMode = false;  // 16 bit UC data in mm else 32F in m,
                                  // for more info -> http://www.ros.org/reps/rep-0118.html
  double mCamMinDepth = 0.2;
  double mCamMaxDepth = 10.0;
  bool mSensCameraSync = false;
  double mSensPubRate = 400.;
  bool mUseOldExtrinsic = false;
  bool mPosTrackingEnabled = false;
  bool mPublishTF = true;
  bool mPublishMapTF = true;
  bool mPublishImuTF = true;
  bool mPoseSmoothing = false;
  bool mAreaMemory = true;
  std::string mAreaMemoryDbPath = "";
  bool mImuFusion = true;
  bool mFloorAlignment = false;
  bool mTwoDMode = false;
  double mFixedZValue = 0.0;
  std::vector<double> mInitialBasePose = std::vector<double>(6, 0.0);
  bool mInitOdomWithPose = true;
  double mPathPubRate = 2.0;
  double mTfOffset = 0.05;
  double mPosTrackDepthMinRange = 0.0;
  bool mSetAsStatic = false;
  bool mSetGravityAsOrigin = false;
  int mPathMaxCount = -1;
  bool mPublishPoseCov = true;
  bool mMappingEnabled = false;
  float mMappingRes = 0.05f;
  float mMappingRangeMax = 10.0f;
  bool mObjDetEnabled = false;
  bool mObjDetTracking = true;
  float mObjDetConfidence = 40.0f;
  double mObjDetPredTimeout = 0.5;
  std::vector<sl::OBJECT_CLASS> mObjDetFilter;
  bool mObjDetPeopleEnable = true;
  bool mObjDetVehiclesEnable = true;
  bool mObjDetBagsEnable = true;
  bool mObjDetAnimalsEnable = true;
  bool mObjDetElectronicsEnable = true;
  bool mObjDetFruitsEnable = true;
  bool mObjDetSportEnable = true;
  bool mObjDetBodyFitting = false;
  sl::BODY_FORMAT mObjDetBodyFmt = sl::BODY_FORMAT::POSE_34;
  sl::DETECTION_MODEL mObjDetModel = sl::DETECTION_MODEL::HUMAN_BODY_FAST;
  sl::OBJECT_FILTERING_MODE mObjFilterMode = sl::OBJECT_FILTERING_MODE::NMS3D;
  // QoS parameters
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS mVideoQos;
  rclcpp::QoS mDepthQos;
  rclcpp::QoS mSensQos;
  rclcpp::QoS mPoseQos;
  rclcpp::QoS mMappingQos;
  rclcpp::QoS mObjDetQos;
  rclcpp::QoS mClickedPtQos;
  // <---- Parameter variables

  // ----> Dynamic params
  OnSetParametersCallbackHandle::SharedPtr mParamChangeCallbackHandle;

  double mPubFrameRate = 15;
  int mCamBrightness = 4;
  int mCamContrast = 4;
  int mCamHue = 0;
  int mCamSaturation = 4;
  int mCamSharpness = 4;
  int mCamGamma = 8;
  bool mCamAutoExpGain = true;
  int mCamGain = 80;
  int mCamExposure = 80;
  bool mCamAutoWB = true;
  int mCamWBTemp = 42;
  int mDepthConf = 50;
  int mDepthTextConf = 100;
  double mPcPubRate = 15.0;
  double mFusedPcPubRate = 1.0;
  bool mRemoveSatAreas = true;
  // <---- Dynamic params

  // ----> Frame IDs
  std::string mRgbFrameId;
  std::string mRgbOptFrameId;

  std::string mDepthFrameId;
  std::string mDepthOptFrameId;

  std::string mDisparityFrameId;
  std::string mDisparityOptFrameId;

  std::string mConfidenceFrameId;
  std::string mConfidenceOptFrameId;

  std::string mCloudFrameId;
  std::string mPointCloudFrameId;

  std::string mMapFrameId = "map";
  std::string mOdomFrameId = "odom";
  std::string mBaseFrameId = "base_link";

  std::string mCameraFrameId;

  std::string mRightCamFrameId;
  std::string mRightCamOptFrameId;
  std::string mLeftCamFrameId;
  std::string mLeftCamOptFrameId;

  std::string mImuFrameId;
  std::string mBaroFrameId;
  std::string mMagFrameId;
  std::string mTempLeftFrameId;
  std::string mTempRightFrameId;
  // <---- Frame IDs

  // ----> Stereolabs Mat Info
  int mCamWidth;   // Camera frame width
  int mCamHeight;  // Camera frame height
  sl::Resolution mMatResol;
  // <---- Stereolabs Mat Info

  // Camera IMU transform
  sl::Transform mSlCamImuTransf;

  // ----> initialization Transform listener
  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  // <---- initialization Transform listener

  // ----> TF Transforms
  tf2::Transform mMap2OdomTransf;       // Coordinates of the odometry frame in map frame
  tf2::Transform mOdom2BaseTransf;      // Coordinates of the base in odometry frame
  tf2::Transform mMap2BaseTransf;       // Coordinates of the base in map frame
  tf2::Transform mSensor2BaseTransf;    // Coordinates of the base frame in sensor frame
  tf2::Transform mSensor2CameraTransf;  // Coordinates of the camera frame in sensor frame
  tf2::Transform mCamera2BaseTransf;    // Coordinates of the base frame in camera frame
  // <---- TF Transforms

  // ----> TF Transforms Flags
  bool mSensor2BaseTransfValid = false;
  bool mSensor2CameraTransfValid = false;
  bool mCamera2BaseTransfValid = false;
  // <---- TF Transforms Flags

  // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
  // Camera infos
  camInfoMsgPtr mRgbCamInfoMsg;
  camInfoMsgPtr mLeftCamInfoMsg;
  camInfoMsgPtr mRightCamInfoMsg;
  camInfoMsgPtr mRgbCamInfoRawMsg;
  camInfoMsgPtr mLeftCamInfoRawMsg;
  camInfoMsgPtr mRightCamInfoRawMsg;
  camInfoMsgPtr mDepthCamInfoMsg;
  // <---- Messages

  // ----> Publishers
  image_transport::CameraPublisher mPubRgb;
  image_transport::CameraPublisher mPubRawRgb;
  image_transport::CameraPublisher mPubLeft;
  image_transport::CameraPublisher mPubRawLeft;
  image_transport::CameraPublisher mPubRight;
  image_transport::CameraPublisher mPubRawRight;
  image_transport::CameraPublisher mPubDepth;
  image_transport::Publisher mPubStereo;
  image_transport::Publisher mPubRawStereo;

  image_transport::CameraPublisher mPubRgbGray;
  image_transport::CameraPublisher mPubRawRgbGray;
  image_transport::CameraPublisher mPubLeftGray;
  image_transport::CameraPublisher mPubRawLeftGray;
  image_transport::CameraPublisher mPubRightGray;
  image_transport::CameraPublisher mPubRawRightGray;

  imagePub mPubConfMap;
  disparityPub mPubDisparity;
  pointcloudPub mPubCloud;
  pointcloudPub mPubFusedCloud;
  posePub mPubPose;
  poseCovPub mPubPoseCov;
  odomPub mPubOdom;
  pathPub mPubOdomPath;
  pathPub mPubPosePath;
  imuPub mPubImu;
  imuPub mPubImuRaw;
  tempPub mPubImuTemp;
  magPub mPubImuMag;
  pressPub mPubPressure;
  tempPub mPubTempL;
  tempPub mPubTempR;
  transfPub mPubCamImuTransf;
  objPub mPubObjDet;
  depthInfoPub mPubDepthInfo;
  planePub mPubPlane;
  markerPub mPubMarker;
  // <---- Publishers

  // <---- Publisher variables
  sl::Timestamp mGrabTS = 0;
  size_t mRgbSubnumber = 0;
  size_t mRgbRawSubnumber = 0;
  size_t mRgbGraySubnumber = 0;
  size_t mRgbGrayRawSubnumber = 0;
  size_t mLeftSubnumber = 0;
  size_t mLeftRawSubnumber = 0;
  size_t mLeftGraySubnumber = 0;
  size_t mLeftGrayRawSubnumber = 0;
  size_t mRightSubnumber = 0;
  size_t mRightRawSubnumber = 0;
  size_t mRightGraySubnumber = 0;
  size_t mRightGrayRawSubnumber = 0;
  size_t mStereoSubnumber = 0;
  size_t mStereoRawSubnumber = 0;
  size_t mDepthSubnumber = 0;
  size_t mConfMapSubnumber = 0;
  size_t mDisparitySubnumber = 0;
  size_t mDepthInfoSubnumber = 0;

  sl::Mat mMatLeft, mMatLeftRaw;
  sl::Mat mMatRight, mMatRightRaw;
  sl::Mat mMatLeftGray, mMatLeftRawGray;
  sl::Mat mMatRightGray, mMatRightRawGray;
  sl::Mat mMatDepth, mMatDisp, mMatConf;
  float mMinDepth = 0.0f;
  float mMaxDepth = 0.0f;
  // <---- Publisher variables

  // ----> Point cloud variables
  sl::Mat mMatCloud;
  sl::FusedPointCloud mFusedPC;
  // <---- Point cloud variables

  // ----> Subscribers
  clickedPtSub mClickedPtSub;
  // <---- Subscribers

  // ----> Threads and Timers
  sl::ERROR_CODE mGrabStatus;
  sl::ERROR_CODE mConnStatus;
  std::thread mGrabThread;        // Main grab thread
  std::thread mVideoDepthThread;  // RGB/Depth data publish thread
  std::thread mPcThread;          // Point Cloud publish thread
  std::thread mSensThread;        // Sensors data publish thread
  bool mThreadStop = false;
  rclcpp::TimerBase::SharedPtr mPathTimer;
  rclcpp::TimerBase::SharedPtr mFusedPcTimer;
  rclcpp::TimerBase::SharedPtr mTempPubTimer;  // Timer to retrieve and publish CMOS temperatures
  // <---- Threads and Timers

  // ----> Thread Sync
  // std::mutex mCloseZedMutex;
  std::mutex mVideoDepthMutex;
  std::mutex mPcMutex;
  std::mutex mRecMutex;
  std::mutex mPosTrkMutex;
  std::mutex mDynParMutex;
  std::mutex mMappingMutex;
  std::mutex mObjDetMutex;
  std::condition_variable mVideoDepthDataReadyCondVar;
  std::condition_variable mPcDataReadyCondVar;
  std::atomic_bool mPcDataReady;
  std::atomic_bool mVideoDepthDataReady;
  // <---- Thread Sync

  // ----> Status Flags
  bool mSvoMode = false;
  bool mSvoPause = false;
  bool mPosTrackingStarted = false;
  bool mVdPublishing = false; // Indicates if video and depth data are subscribed and then published
  bool mPcPublishing = false; // Indicates if point cloud data are subscribed and then published
  bool mTriggerAutoExpGain = true;  // Triggered on start
  bool mTriggerAutoWB = true;       // Triggered on start
  bool mRecording = false;
  sl::RecordingStatus mRecStatus = sl::RecordingStatus();
  bool mPosTrackingReady = false;
  sl::POSITIONAL_TRACKING_STATE mPosTrackingStatus;
  bool mResetOdom = false;
  bool mMappingRunning = false;
  bool mObjDetRunning = false;
  bool mRgbSubscribed = false;
  // <---- Status Flags


  // ----> Positional Tracking
  sl::Pose mLastZedPose;  // Sensor to Map transform
  sl::Transform mInitialPoseSl;
  std::vector<geometry_msgs::msg::PoseStamped> mOdomPath;
  std::vector<geometry_msgs::msg::PoseStamped> mMapPath;
  // <---- Positional Tracking

  // Diagnostic
  float mTempLeft = -273.15f;
  float mTempRight = -273.15f;
  std::unique_ptr<sl_tools::WinAvg> mElabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mGrabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mVideoDepthPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mVideoDepthElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPcPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPcProcMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mImuPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mBaroPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mMagPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mObjDetPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mObjDetElabMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubFusedCloudPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubOdomTF_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubPoseTF_sec;
  std::unique_ptr<sl_tools::WinAvg> mPubImuTF_sec;
  bool mImuPublishing = false;
  bool mMagPublishing = false;
  bool mBaroPublishing = false;
  bool mObjDetSubscribed = false;

  diagnostic_updater::Updater mDiagUpdater;  // Diagnostic Updater

  // ----> Timestamps
  rclcpp::Time mFrameTimestamp;
  // <---- Timestamps

  // ----> SVO Recording parameters
  unsigned int mSvoRecBitrate = 0;
  sl::SVO_COMPRESSION_MODE mSvoRecCompr = sl::SVO_COMPRESSION_MODE::H264;
  unsigned int mSvoRecFramerate = 0;
  bool mSvoRecTranscode = false;
  std::string mSvoRecFilename;
  // <---- SVO Recording parameters

  // ----> Services
  resetOdomSrvPtr mResetOdomSrv;
  resetPosTrkSrvPtr mResetPosTrkSrv;
  setPoseSrvPtr mSetPoseSrv;
  enableObjDetPtr mEnableObjDetSrv;
  enableMappingPtr mEnableMappingSrv;
  startSvoRecSrvPtr mStartSvoRecSrv;
  stopSvoRecSrvPtr mStopSvoRecSrv;
  pauseSvoSrvPtr mPauseSvoSrv;
  setRoiSrvPtr mSetRoiSrv;
  resetRoiSrvPtr mResetRoiSrv;
  // <---- Services

  // ----> Services names
  const std::string mSrvResetOdomName = "reset_odometry";
  const std::string mSrvResetPoseName = "reset_pos_tracking";
  const std::string mSrvSetPoseName = "set_pose";
  const std::string mSrvEnableObjDetName = "enable_obj_det";
  const std::string mSrvEnableMappingName = "enable_mapping";
  const std::string mSrvStartSvoRecName = "start_svo_rec";
  const std::string mSrvStopSvoRecName = "stop_svo_rec";
  const std::string mSrvToggleSvoPauseName = "toggle_svo_pause";
  const std::string mSrvSetRoiName = "set_roi";
  const std::string mSrvResetRoiName = "reset_roi";
  // <---- Services names
};

}  // namespace stereolabs

#endif  // ZED_CAMERA_COMPONENT_HPP_
