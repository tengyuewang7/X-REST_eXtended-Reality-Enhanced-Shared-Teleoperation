using JarvenFramework.ResModule;
using Microsoft.MixedReality.Toolkit;
using UnityEngine;

public class ToggleSwitch : MonoBehaviour
{
    // Boolean variable to represent the toggle switch state
    public bool groundToggle = false;
    public bool rizonToggle = false;
    public bool hanoiGroupToggle = false;
    public bool rosCommunicationToggle = false;
    public bool hanoiGuidanceToggle = false;
    public bool realsenseImageToggle = false;
    public bool pointCloudToggle = false;
    public bool endeffectorControllerToggle = false;
    public bool gripperToggle = false;
    public bool suctionCheckToggle = false;
    public bool fixOrientationToggle = false;

    // Variable to keep a reference to the instantiated GameObject
    private GameObject ground; 
    private GameObject rizon;
    private GameObject hanoiGroup;
    private GameObject realsenseImage;
    private GameObject pointCloud; 
    private GameObject endeffectorController;
    private GameObject rosCommunication;

    // Method to call when the toggle is turned on
    public void ToggleGround()
    {
        groundToggle = !groundToggle; 
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Ground.prefab", (obj) =>
        {
            if (ground == null)
            {
                ground = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
                ground.name = "Ground";
            }  
            else { Destroy(ground); }
        }); 
    } 

    public void ToggleRizon()
    {
        rizonToggle = !rizonToggle; 
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Rizon.prefab", (obj) =>
        {
            if (rizon == null)
            {
                rizon = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
                rizon.name = "Rizon";
                // rizon.AddComponent<RizonStatesSubscriber>();
                // rizon.AddComponent<TrajectoryVisualization>();
                // AddController();
            }
            else { Destroy(rizon); }
        });
    }

    public void ToggleHanoiGroup()
    {
        hanoiGroupToggle = !hanoiGroupToggle; 
        if (hanoiGroup == null)
        {
            ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/HanoiGroup.prefab", (obj) =>
            {
                hanoiGroup = GameObject.Instantiate(obj, new Vector3(0.4f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
                hanoiGroup.name = "HanoiGroup";  
                hanoiGroup.AddComponent<DrawBaseLine>();
                hanoiGroup.AddComponent<BaseAttraction>();
                hanoiGroup.AddComponent<FailureDetection>();

                // hanoiGroup.AddComponent<HanoiVisualizationGuidance>();
                // hanoiGroup.AddComponent<HanoiGrabPublisher>();
                // hanoiPosePublisher = hanoiGroup.AddComponent<HanoiPosePublisher>();

                // hanoiGroup.AddComponent<HanoiVisualDemo>();
                // hanoiGroup.AddComponent<SolveHanoi>();
            });
        }
        else { Destroy(hanoiGroup); }
    }

    public void ToggleHanoiGuidance()
    {
        hanoiGuidanceToggle = !hanoiGuidanceToggle; 
        if (hanoiGroup.GetComponent<HanoiVisualizationGuidance>() == null)
        {
            hanoiGroup.AddComponent<HanoiVisualizationGuidance>();
        }
        else
        {
            HanoiVisualizationGuidance component = hanoiGroup.GetComponent<HanoiVisualizationGuidance>(); 
            Destroy(component);
            Transform guidanceGameObject = hanoiGroup.transform.Find("HanoiVisualGuidance");
            Destroy(guidanceGameObject.gameObject);
        }
    }

    public void ToggleRealsenseImage()
    {
        realsenseImageToggle = !realsenseImageToggle; 
        if (realsenseImage == null)
        {
            ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/CameraImage.prefab", (obj) =>
            {
                realsenseImage = GameObject.Instantiate(obj, new Vector3(0.3524f, 0.1563f, 0.1172f), Quaternion.Euler(0, 150, 0));
                realsenseImage.name = "RealsenseImage"; 
            });
        }
        else { Destroy(realsenseImage); }
    }

    public void TogglePointCloud()
    {
        pointCloudToggle = !pointCloudToggle; 
        GameObject.Find("PointCloudVisualization").GetComponent<PointCloudRenderer>().SetVisibility(); 
    }

    public void ToggleEndeffectorController()
    {
        endeffectorControllerToggle = !endeffectorControllerToggle; 
        if (endeffectorController == null)
        {
            string flangePath = "Rizon/world/base_link/link_1/link_2/link_3/link_4/link_5/link_6/link_7/flange";
            Transform flangeTransform = GameObject.Find(flangePath).transform;
            Vector3 pos = flangeTransform.position;
            Quaternion rot = flangeTransform.rotation * Quaternion.Euler(-90f, 0f, 0f);
            
            ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/EndEffectorController.prefab", (obj) =>
            {
                endeffectorController = GameObject.Instantiate(obj, pos, rot);
                endeffectorController.name = "EndeffectorController";
                endeffectorController.transform.SetParent(rizon.transform); 
            });
        }
        else { Destroy(endeffectorController); }
    }

    // Method to call when the toggle is turned on
    public void ToggleRosCommunication()
    {
        rosCommunicationToggle = !rosCommunicationToggle; 
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/ROSCommunication.prefab", (obj) =>
        {
            if (rosCommunication == null)
            {
                rosCommunication = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
                rosCommunication.name = "RosCommunication";
            }
            else { Destroy(rosCommunication); }
        });
    }

    // Method to call when the toggle is turned on
    public void ToggleSuctionCheck()
    {
        suctionCheckToggle = !suctionCheckToggle;
        SuctionCheck tempSuctionCheck = transform.GetComponent<SuctionCheck>();
        tempSuctionCheck.enabled = !tempSuctionCheck.enabled; 
    }

    // Method to call when the toggle is turned on
    public void ToggleGripper()
    {
        gripperToggle = !gripperToggle; 
    }
    public void ToggleFixOrientation()
    {
        fixOrientationToggle = !fixOrientationToggle;
    }
}

