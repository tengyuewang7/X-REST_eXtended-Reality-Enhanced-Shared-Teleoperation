using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using static UnityEditor.PlayerSettings;
using RosMessageTypes.Flexiv;
using RosMessageTypes.Sensor;


public class LatencyTest : MonoBehaviour
{
    private ROSConnection ros;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<StringMsg>("latency_test_unity");
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("latency_test_ros2", ReceiveCallback);
    }

    private void ReceiveCallback(StringMsg msg)
    {
        StringMsg msg2 = msg;
        ros.Publish("latency_test_unity", msg2);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        
    }
}
