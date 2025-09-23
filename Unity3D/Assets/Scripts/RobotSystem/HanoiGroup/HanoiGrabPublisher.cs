using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Flexiv;

public class HanoiGrabPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "hanoi_grab";
    public float publishMessageFrequency = 0.01f;
    private HanoiGrabMsg message;
    private float timeElapsed; 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<HanoiGrabMsg>(topicName);
        message = new HanoiGrabMsg();
    }   

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            message.is_grab[0] = GetComponent<HanoiGroup>().isGrab[0];
            message.is_grab[1] = GetComponent<HanoiGroup>().isGrab[1];
            message.is_grab[2] = GetComponent<HanoiGroup>().isGrab[2];
            ros.Publish(topicName, message);
            timeElapsed = 0;
        }
    }
}
