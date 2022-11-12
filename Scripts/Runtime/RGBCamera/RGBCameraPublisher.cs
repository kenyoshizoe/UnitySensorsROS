using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(Camera))]
[RequireComponent(typeof(FRJ.Sensor.RGBCamera))]
public class RGBCameraPublisher : MonoBehaviour
{

  [SerializeField] private string _topicName = "image";
  [SerializeField] private string _frameId   = "camera";

  private string _imageTopicName = "";
  private string _cameraInfoTopicName = "";

  private float _timeElapsed = 0f;
  private float _timeStamp   = 0f;

  private ROSConnection _ros;
  private CompressedImageMsg _imageMsg;
  private CameraInfoMsg _cameraInfoMsg;

  private FRJ.Sensor.RGBCamera _camera;

  void Start()
  {
    // Get Rotate Lidar
    this._camera = GetComponent<FRJ.Sensor.RGBCamera>();
    this._camera.Init();

    // setup ROS
    this._ros = ROSConnection.instance;
    this._imageTopicName = this._topicName + "/compressed";
    this._cameraInfoTopicName = this._topicName + "/camera_info";

    this._ros.RegisterPublisher<CompressedImageMsg>(this._imageTopicName);
    this._ros.RegisterPublisher<CameraInfoMsg>(this._cameraInfoTopicName);

    // setup ROS Message
    this._imageMsg = new CompressedImageMsg();
    this._imageMsg.header.frame_id = this._frameId;
    this._imageMsg.format = "jpeg";

    this._cameraInfoMsg = new CameraInfoMsg();
    this._cameraInfoMsg.header.frame_id = this._frameId;
    this._cameraInfoMsg.height = this._camera.height;
    this._cameraInfoMsg.width = this._camera.width;
    this._cameraInfoMsg.distortion_model = "plumb_bob";
    this._cameraInfoMsg.D = new double[] { 0, 0, 0, 0, 0 };

    double HFOV = Camera.VerticalToHorizontalFieldOfView(
        GetComponent<Camera>().fieldOfView, GetComponent<Camera>().aspect);
    double focalLength = this._camera.width / (2.0 * Math.Tan(HFOV * Mathf.Deg2Rad / 2.0));

    this._cameraInfoMsg.K = new double[] {
        focalLength, 0, (this._camera.width + 1) / 2.0,
        0, focalLength, (this._camera.height + 1) / 2.0,
        0, 0, 1
    };
    this._cameraInfoMsg.P = new double[] {
        focalLength, 0, (this._camera.width + 1) / 2.0, 0,
        0, focalLength, (this._camera.height + 1) / 2.0, 0,
        0, 0, 1, 0
    };
  }

    void Update()
    {
        this._timeElapsed += Time.deltaTime;

        if(this._timeElapsed > (1f/this._camera.scanRate))
        {
            // Update ROS Message
# if ROS2
            int sec = (int)Math.Truncate(this._timeStamp);
# else
            uint sec = (uint)Math.Truncate(this._timeStamp);
# endif
            uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );

            this._imageMsg.header.stamp.sec = sec;
            this._imageMsg.header.stamp.nanosec = nanosec;
            this._imageMsg.data = this._camera.data;
            this._ros.Send(this._imageTopicName, this._imageMsg);

            this._cameraInfoMsg.header.stamp.sec = sec;
            this._cameraInfoMsg.header.stamp.nanosec = nanosec;
            this._ros.Send(this._cameraInfoTopicName, this._cameraInfoMsg);

            // Update time
            this._timeElapsed = 0;
            this._timeStamp = Time.time;
        }
    }
}
