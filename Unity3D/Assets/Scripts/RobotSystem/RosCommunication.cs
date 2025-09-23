using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Flexiv;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class RosCommunication : MonoBehaviour
{
    private GameObject rizon; 
    private ROSConnection ros;
    public float publishMessageFrequency = 0.01f;
    private float timeElapsed;

    public List<string> subscribedTopics = new List<string>();
    public List<string> publishedTopics = new List<string>();

    private string subTopic_1 = "robot_states";
    private string subTopic_2 = "point_cloud";
    private string subTopic_3 = "camera_images"; 

    private string pubTopic_1 = "hanoi_states";
    private HanoiGroup hanoiGroup;
    private List<Transform> hanoiRelativeTransform = new List<Transform>();
    private List<Transform> staticHanoiRelativeTransform = new List<Transform>();

    private string pubTopic_3 = "is_manual";
    private bool isManual; 

    private string pubTopic_4 = "endeffector_pose";
    private GameObject endeffectorRelativetTransform; 
    private Transform endeffectorTransform;
    private ToggleSwitch toggleSwitch; 

    [HideInInspector]
    public float[] jointState;
    public bool is_sucked; 
    [HideInInspector]
    public Material color_indicator;
    [HideInInspector]
    public List<Vector3> pointCloudPositionList;
    [HideInInspector]
    public List<Color> pointCloudColorList;
    public int validPointCount = 0; 
    [HideInInspector]
    public HanoiStatesMsg hanoiStatesMsg = new HanoiStatesMsg();
    [HideInInspector]
    public Material imageMaterial;
    [HideInInspector]
    public StringMsg modeMsg = new StringMsg();
    [HideInInspector]
    public PoseStampedMsg endeffectorControllerPoseMsg = new PoseStampedMsg(); 

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance(); 
        rizon = GameObject.Find("Rizon"); 
        if (rizon == null )
        {
            Debug.LogError("Rizon GameObject not found!");
            return;
        }

        ROSConnection.GetOrCreateInstance().Subscribe<RobotStatesMsg>(subTopic_1, ReceiveCallback_1);
        color_indicator = rizon.transform.Find("world/base_link/link_1/link_2/link_3/link_4/link_5/link_6/link_7/flange/sucker/sucker_disc").GetComponent<Renderer>().material; 
        jointState = new float[7];
        rizon.GetComponent<JointController>()._jointState = jointState;
        is_sucked = false; 
        subscribedTopics.Add(subTopic_1);

        ROSConnection.GetOrCreateInstance().Subscribe<PointCloud2Msg>(subTopic_2, ReceiveCallback_2);
        pointCloudPositionList = GameObject.Find("PointCloudVisualization")?.GetComponent<PointCloudRenderer>().pointCloudPositionList;
        pointCloudColorList = GameObject.Find("PointCloudVisualization")?.GetComponent<PointCloudRenderer>().pointCloudColorList;
        subscribedTopics.Add(subTopic_2); 

        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(subTopic_3, ReceiveCallback_3);
        imageMaterial = GameObject.Find("RealsenseImage")?.GetComponent<MeshRenderer>().materials[0];
        subscribedTopics.Add(subTopic_3); 

        ros.RegisterPublisher<HanoiStatesMsg>(pubTopic_1);
        hanoiGroup = GameObject.Find("HanoiGroup")?.GetComponent<HanoiGroup>();
        publishedTopics.Add(pubTopic_1);
        for (int i = 0; i < hanoiGroup.hanois.Length; i++)
        {
            hanoiRelativeTransform.Add(new GameObject("hanoiRelativeTransform").transform);
            hanoiRelativeTransform[i].SetParent(rizon.transform); 
            staticHanoiRelativeTransform.Add(new GameObject("staticHanoiRelativeTransform").transform);
            staticHanoiRelativeTransform[i].SetParent(rizon.transform);
        }

        ros.RegisterPublisher<StringMsg>(pubTopic_3);
        publishedTopics.Add(pubTopic_3);

        ros.RegisterPublisher<PoseStampedMsg>(pubTopic_4);
        endeffectorRelativetTransform = new GameObject("endeffectorRelativetTransform");
        endeffectorRelativetTransform.transform.SetParent(rizon.transform); 
        endeffectorTransform = rizon.transform.Find("EndeffectorController")?.transform;
        toggleSwitch= GameObject.Find("Master").GetComponents<ToggleSwitch>()[0];
        publishedTopics.Add(pubTopic_4);

    }

    private void ReceiveCallback_1(RobotStatesMsg msg)
    {
        for (int i = 0; i < jointState.Length; i++) { jointState[i] = (float)msg.q[i]; }
        is_sucked = msg.digital_input_0; 
        Color clr = new Color(1f, 0.5f, 0f, 0.8f); 
        if (msg.header.frame_id == "none") { clr = Color.white; } 
        else if (msg.header.frame_id == "smooth") { clr = Color.blue; } 
        else if (msg.header.frame_id == "tracking") { clr = Color.green; } 
        else if (msg.header.frame_id == "avoiding") { clr = Color.red; }
        else if (msg.header.frame_id == "manual") { clr = Color.yellow; }
        color_indicator.color = Color.white; 
    }

    private void ReceiveCallback_2(PointCloud2Msg msg)
    {
        int pointStep = (int)msg.point_step;
        int pointCount = (int)(msg.data.Length / pointStep);
        validPointCount = pointCount; 

        if (pointCloudPositionList.Capacity < pointCount)
        {
            int additionalCapacity = pointCount - pointCloudPositionList.Capacity;
            pointCloudPositionList.Capacity = pointCount;
            pointCloudColorList.Capacity = pointCount; 
            for (int i = 0; i < additionalCapacity; i++)
            {
                pointCloudPositionList.Add(Vector3.zero);
                pointCloudColorList.Add(Color.white); 
            }
        }
        GCHandle handle = GCHandle.Alloc(msg.data, GCHandleType.Pinned);
        IntPtr pointer = handle.AddrOfPinnedObject();

        for (int i = 0; i < pointCount; i++)
        {
            int baseIndex = i * pointStep;

            if (BitConverter.ToSingle(msg.data, baseIndex + 8) < 0.01)
            {
                continue; 
            }

            float x = BitConverter.ToSingle(msg.data, baseIndex);
            float y = BitConverter.ToSingle(msg.data, baseIndex + 4);
            float z = BitConverter.ToSingle(msg.data, baseIndex + 8);
            float r = BitConverter.ToSingle(msg.data, baseIndex + 12);
            float g = BitConverter.ToSingle(msg.data, baseIndex + 16);
            float b = BitConverter.ToSingle(msg.data, baseIndex + 20);

            // Convert from ROS FLU to Unity
            PointMsg rosPoint = new PointMsg(x, y, z);
            pointCloudPositionList[i] = rosPoint.From<FLU>();
            pointCloudColorList[i] = new Color(r, g, b); 

        }
        handle.Free();
    }

    private void ReceiveCallback_3(ImageMsg msg)
    {
        if (imageMaterial != null) { imageMaterial.mainTexture = msg.ToTexture2D(); } 
        else { imageMaterial = GameObject.Find("RealsenseImage")?.GetComponent<MeshRenderer>().materials[0]; }


    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime; 

        if (timeElapsed > publishMessageFrequency)
        {
            // checked if there is hanoiGroup
            if (hanoiGroup != null)
            {
                hanoiStatesMsg.is_grab[0] = hanoiGroup.isGrab[0];
                hanoiStatesMsg.is_grab[1] = hanoiGroup.isGrab[1];
                hanoiStatesMsg.is_grab[2] = hanoiGroup.isGrab[2];
                for (int i = 0; i < hanoiGroup.hanois.Length; i++)
                {
                    // Assign the grab info
                    hanoiStatesMsg.is_grab[i] = hanoiGroup.isGrab[i];

                    // Assign the current pose
                    hanoiRelativeTransform[i].position = rizon.transform.InverseTransformPoint(hanoiGroup.hanois[i].transform.position);
                    hanoiRelativeTransform[i].rotation = Quaternion.Inverse(rizon.transform.rotation) * hanoiGroup.hanois[i].transform.rotation;
                    hanoiRelativeTransform[i].Rotate(new Vector3(0f, 180f, 180f));
                    hanoiStatesMsg.current_pose[i].position = hanoiRelativeTransform[i].position.To<FLU>();
                    hanoiStatesMsg.current_pose[i].orientation = hanoiRelativeTransform[i].rotation.To<FLU>();

                    // Assign the static pose
                    staticHanoiRelativeTransform[i].position = rizon.transform.InverseTransformPoint(hanoiGroup.staticHanoiPosition[i]);
                    hanoiStatesMsg.static_pose[i].position = staticHanoiRelativeTransform[i].position.To<FLU>();
                    hanoiStatesMsg.static_pose[i].orientation = new QuaternionMsg(0, 1, 0, 0);
                }
                ros.Publish(pubTopic_1, hanoiStatesMsg);
            }
            else
            {
                hanoiGroup = GameObject.Find("HanoiGroup")?.GetComponent<HanoiGroup>();
            }

            // Publish pubTopic_3: control_mode 
            if (endeffectorTransform != null) { modeMsg.data = "manual"; isManual = true; } 
            else { modeMsg.data = "shared"; isManual = false; }
            ros.Publish(pubTopic_3, modeMsg);

            // Publish pubTopic_4: endeffector_controller_pose
            if (endeffectorTransform != null)
            {
                if (toggleSwitch.gripperToggle) { endeffectorControllerPoseMsg.header.frame_id = "true"; }
                else { endeffectorControllerPoseMsg.header.frame_id = "false"; }

                if (toggleSwitch.fixOrientationToggle)
                {
                    endeffectorTransform.rotation = new Quaternion(0f, 0f, 0f, -1f);
                }

                endeffectorRelativetTransform.transform.position = rizon.transform.InverseTransformPoint(endeffectorTransform.position);
                endeffectorRelativetTransform.transform.rotation = Quaternion.Inverse(rizon.transform.rotation) * endeffectorTransform.rotation;
                endeffectorRelativetTransform.transform.Rotate(new Vector3(0f, 180f, 180f));
                endeffectorControllerPoseMsg.pose.position = endeffectorRelativetTransform.transform.position.To<FLU>(); 
                endeffectorControllerPoseMsg.pose.orientation = endeffectorRelativetTransform.transform.rotation.To<FLU>();

                ros.Publish(pubTopic_4, endeffectorControllerPoseMsg);
            }
            else
            {
                endeffectorTransform = rizon.transform.Find("EndeffectorController")?.transform;
            }

            timeElapsed = 0;
        }
    }
}
