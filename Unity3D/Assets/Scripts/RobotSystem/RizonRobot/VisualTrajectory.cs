using RosMessageTypes.Flexiv;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.UnityRoboticsDemo;

public class VisualTrajectory : MonoBehaviour
{
    string m_ServiceName = "spawn";


    void Start()
    {
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnRizonVisualRequest, SpawnRizonVisualResponse>(m_ServiceName, SpawnARizon);

    }

    private SpawnRizonVisualResponse SpawnARizon(SpawnRizonVisualRequest request)
    {
        Debug.Log("Received request for object: " + request.joint_state.Length);
        SpawnRizonVisualResponse response = new SpawnRizonVisualResponse();
        response.success = true; 
        return response; 
    }

}
