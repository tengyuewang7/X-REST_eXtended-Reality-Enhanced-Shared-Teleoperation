using UnityEngine;
using System.Collections;
using JarvenFramework.ResModule;
using System.Collections.Generic;
using Unity.XR.CoreUtils;
using Microsoft.MixedReality.Toolkit.UI;
using UniRx.Triggers;
using UnityEngine.SceneManagement;

public class StartManual : MonoBehaviour
{
    private GameObject ground;
    private GameObject rizon;
    private GameObject hanoiGroup;
    private GameObject controller;
    private GameObject coordinateCalibration;
    private GameObject modeSwitch;

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
            AddController();
        });  

    }


    void AddController()
    {
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/EndEffectorController.prefab", (obj) =>
        {
            controller = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f), rizon.transform);
            controller.name = "EndeffectorController";
            controller.GetComponent<MeshRenderer>().enabled = false; 
        });
    }
    
    bool isManualMode = false;
    public void SwitchOperationMode()
    {
        isManualMode = !isManualMode;
    }
    
    // Update is called once per frame
    void Update()
    {
        if (!isManualMode) { SceneManager.LoadScene("SampleScene"); }
    }
}
