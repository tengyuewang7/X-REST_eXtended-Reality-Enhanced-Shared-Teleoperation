using UnityEngine;
using System.Collections;
using JarvenFramework.ResModule;
using System.Collections.Generic;
using Unity.XR.CoreUtils;
using Microsoft.MixedReality.Toolkit.UI;
using UniRx.Triggers;
using UnityEngine.SceneManagement;

public class StartWorld : MonoBehaviour
{
    private GameObject ground;
    private GameObject rizon;
    private GameObject hanoiGroup;
    private GameObject controller;
    private GameObject realSenseImage;
    private GameObject coordinateCalibration;
    private GameObject modeSwitch;
    private HanoiPosePublisher hanoiPosePublisher;
    private bool isManualMode = false;


    // Start is called before the first frame update
    void Start()
    {
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Ground.prefab", (obj) =>
        {
            ground = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
            ground.name = "Ground";
        });

        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Rizon.prefab", (obj) =>
        {
            rizon = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
            rizon.name = "Rizon";
            rizon.AddComponent<RizonStatesSubscriber>();

            // rizon.AddComponent<TrajectoryVisualization>();
            // AddController();
        });  

        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/HanoiGroup.prefab", (obj) =>
        {
            hanoiGroup = GameObject.Instantiate(obj, new Vector3(0.4f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
            hanoiGroup.AddComponent<DrawBaseLine>();
            hanoiGroup.AddComponent<DrawBaseLine>();
            hanoiGroup.AddComponent<FailureDetection>();
            hanoiGroup.AddComponent<HanoiVisualizationGuidance>();
            hanoiGroup.AddComponent<HanoiGrabPublisher>();
            hanoiPosePublisher = hanoiGroup.AddComponent<HanoiPosePublisher>();
            // hanoiGroup.AddComponent<HanoiVisualDemo>();
            // hanoiGroup.AddComponent<SolveHanoi>();
        });


        AddRealsenseImage();

        /*
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Switch.prefab", (obj) =>
        {
            modeSwitch = GameObject.Instantiate(obj, new Vector3(0.4f, 0f, 0.1f), new Quaternion(0f, 0f, 0f, 1f));
            Debug.Log("here");
            modeSwitch.GetComponent<Interactable>().OnClick.AddListener(SwitchOperationMode);
        }); */


        /*
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Coordinate Calibration.prefab", (obj) =>
        {
            coordinateCalibration = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
            Move move = coordinateCalibration.GetComponent<Move>();
            move.referenceObject = rizon;
            move.objectList.Add(ground);
            move.objectList.Add(rizon);
            move.objectList.Add(hanoiGroup);
        }); */


    }


    void AddController(bool goToFlange = false)
    {
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/EndEffectorController.prefab", (obj) =>
        {
            controller = GameObject.Instantiate(obj, new Vector3(0f, 0.5f, 0.6f), new Quaternion(0f, 0f, 0f, 1f), rizon.transform);

            controller.name = "EndeffectorController";
            controller.GetComponent<MeshRenderer>().enabled = true; 
        });
    }

    void AddRealsenseImage()
    {
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/CameraImage.prefab", (obj) =>
        {
            realSenseImage = GameObject.Instantiate(obj, new Vector3(0.3f, 0f, 0.8f), new Quaternion(0f, 0f, 0f, 1f));
        });
    }

    public void SwitchOperationMode()
    {
        isManualMode = !isManualMode;
        if(isManualMode)
        {
            AddController(true);
            AddRealsenseImage();
            hanoiPosePublisher.enabled = false; 
        } 
        else
        {
            Destroy(controller);
            Destroy(realSenseImage); 
            hanoiPosePublisher.enabled = true;
        }

    }
    
    // Update is called once per frame
    void Update()
    {
    }
}
