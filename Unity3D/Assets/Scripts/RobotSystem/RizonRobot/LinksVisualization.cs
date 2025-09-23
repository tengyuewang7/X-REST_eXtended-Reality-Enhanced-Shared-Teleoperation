using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Flexiv;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using JarvenFramework.ResModule;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Cysharp.Threading.Tasks.Triggers;

public class LinksVisualization : MonoBehaviour
{
    private string _topicName = "key_frames";
    private GameObject rizonFrames;
    private List<GameObject> rizonFramesList;

    private GameObject gg;

    // Start is called before the first frame update
    void Start()
    {
        rizonFrames = new GameObject("RizonFrames");
        rizonFramesList = new List<GameObject>();

        ROSConnection.GetOrCreateInstance().Subscribe<RizonFrameMsg>(_topicName, ReceiveCallback);

        for (int i = 0; i < 8; i++)
        {
            ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/XYZCoordinate.prefab", (obj) =>
            {
                GameObject frame = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
                frame.transform.localScale = new Vector3((float)0.5 * frame.transform.localScale.x, (float)0.5 * frame.transform.localScale.y,
                    (float)0.5 * frame.transform.localScale.z);
                frame.transform.parent = rizonFrames.transform;
                rizonFramesList.Add(frame);
            });
        }
    }

    private void ReceiveCallback(RizonFrameMsg msg)
    {

        for (int i = 0; i < rizonFramesList.Count; i++)
        {
            rizonFramesList[i].transform.position = new Vector3<FLU>((float)msg.frames[i].data[0], (float)msg.frames[i].data[1], (float)msg.frames[i].data[2]).toUnity;

            rizonFramesList[i].transform.position = new Vector3(rizonFramesList[i].transform.position.x - 0.15f, rizonFramesList[i].transform.position.y,
                rizonFramesList[i].transform.position.z);

            rizonFramesList[i].transform.rotation = new Quaternion<FLU>((float)msg.frames[i].data[3], 
                (float)msg.frames[i].data[4], (float)msg.frames[i].data[5], (float)msg.frames[i].data[6]).toUnity;

            rizonFramesList[i].name = msg.frames[i].name; 
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
