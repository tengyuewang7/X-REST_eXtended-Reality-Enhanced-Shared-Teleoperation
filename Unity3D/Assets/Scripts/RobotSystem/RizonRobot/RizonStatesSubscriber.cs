using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using robotStates = RosMessageTypes.Flexiv.RobotStatesMsg;

public class RizonStatesSubscriber : MonoBehaviour
{
    private string _topicName = "robot_states";
    [HideInInspector]
    public float[] _jointState = { 0, 0, 0, 0, 0, 0, 0 };

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<robotStates>(_topicName, ReceiveCallback);
    }
    private void ReceiveCallback(robotStates msg)
    {

        for (int i = 0; i < _jointState.Length; i++)
        {
            _jointState[i] = (float)msg.q[i];
        }
    }
}
