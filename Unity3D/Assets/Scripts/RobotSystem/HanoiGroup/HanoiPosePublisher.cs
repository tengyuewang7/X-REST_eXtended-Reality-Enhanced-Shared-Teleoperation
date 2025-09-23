using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Flexiv;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class HanoiPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "hanoi_pose"; 
    public float publishMessageFrequency = 0.01f;
    private HanoiPoseMsg message;
    private GameObject robotBase; 
    private List<Transform> hanoiRelativeTransform = new List<Transform>();
    private float timeElapsed;

    private HanoiGroup hanoiInfo; 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance(); 
        ros.RegisterPublisher<HanoiPoseMsg>(topicName);

        robotBase = GameObject.Find("Rizon"); 
        if (robotBase == null )
        {
            Debug.LogError("Not Find Rizon!");
        }
        hanoiRelativeTransform.Add(new GameObject("hanoiRelativeTransform").transform);
        hanoiRelativeTransform.Add(new GameObject("hanoiRelativeTransform").transform);
        hanoiRelativeTransform.Add(new GameObject("hanoiRelativeTransform").transform);
        hanoiRelativeTransform[0].SetParent(robotBase.transform);
        hanoiRelativeTransform[1].SetParent(robotBase.transform);
        hanoiRelativeTransform[2].SetParent(robotBase.transform);

        message = new HanoiPoseMsg();
        message.pose_array[0] = new RosMessageTypes.Geometry.PoseMsg();
        message.pose_array[1] = new RosMessageTypes.Geometry.PoseMsg();
        message.pose_array[2] = new RosMessageTypes.Geometry.PoseMsg();

        hanoiInfo = GetComponent<HanoiGroup>();

    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            for (int i =0; i < hanoiRelativeTransform.Count; i++)
            {

                hanoiRelativeTransform[i].position = robotBase.transform.InverseTransformPoint(hanoiInfo.hanois[i].transform.position);
                hanoiRelativeTransform[i].rotation = Quaternion.Inverse(robotBase.transform.rotation) * hanoiInfo.hanois[i].transform.rotation;
                hanoiRelativeTransform[i].Rotate(new Vector3(0f, 180f, 180f));
            }

            UpdateMessage();

            timeElapsed = 0;
        }
    }

    void UpdateMessage()
    {
        for (int i = 0; i < hanoiInfo.hanois.Length; i++)
        {
            message.pose_array[i].position = hanoiRelativeTransform[i].position.To<FLU>();
            message.pose_array[i].orientation = hanoiRelativeTransform[i].rotation.To<FLU>();

            ros.Publish(topicName, message); 
        }
    }
}
