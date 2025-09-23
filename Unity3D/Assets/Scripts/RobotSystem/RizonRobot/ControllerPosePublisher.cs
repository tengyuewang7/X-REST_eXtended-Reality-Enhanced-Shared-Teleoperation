using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System;
using System.Collections;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


/// <summary>
/// 
/// </summary>
public class ControllerPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    private string topicName = "target_pose";
    private float publishMessageFrequency = 0.01f;
    private string FrameId = "controller";
    private GameObject robotBase;

    private PoseStampedMsg message;
    private Transform relativeTransform;
    private DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    private float timeElapsed;

    void Start()
    {
        // transform.position = new Vector3(0f, 0.5f, 0.6f);
        // transform.rotation = new Quaternion(0f, 0f, 0f, 1f);

        robotBase = transform.parent.gameObject;
        relativeTransform = new GameObject("RelativeTransform").transform;
        relativeTransform.SetParent(robotBase.transform);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);

        message = new PoseStampedMsg(); 
        message.header.frame_id = FrameId; 

        TimeSpan timeSinceEpoch = DateTime.UtcNow.Subtract(unixEpoch);
        message.header.stamp.sec = (int)timeSinceEpoch.TotalSeconds;
        message.header.stamp.nanosec = (uint)timeSinceEpoch.Milliseconds * 1000;  // DateTime cannot provide accurate time scale to nanosecond.
    }



    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            relativeTransform.position = robotBase.transform.InverseTransformPoint(transform.position);
            relativeTransform.rotation = Quaternion.Inverse(robotBase.transform.rotation) * transform.rotation;
            relativeTransform.Rotate(new Vector3(0f, 180f, 180f));

            UpdateMessage(relativeTransform);

            timeElapsed = 0;
        }
    }

    private void UpdateMessage(Transform trans)
    {
        TimeSpan timeSinceEpoch = DateTime.UtcNow.Subtract(unixEpoch);
        message.header.stamp.sec = (int)timeSinceEpoch.TotalSeconds;
        message.header.stamp.nanosec = (uint)timeSinceEpoch.Milliseconds * 1000;

        message.pose.position = trans.position.To<FLU>();
        message.pose.orientation = trans.rotation.To<FLU>(); 

        ros.Publish(topicName, message);
    }
}
