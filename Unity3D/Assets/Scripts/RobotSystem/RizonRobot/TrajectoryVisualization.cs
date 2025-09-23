using UnityEngine;
using System.Collections;
using JarvenFramework.ResModule;
using System.Collections.Generic;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Flexiv;

public class TrajectoryVisualization : MonoBehaviour
{
    [SerializeField]
    public string m_ServiceName = "spawn_rizon_visual";

    private List<GameObject> rizonList = new List<GameObject>();
    private int countRizon = 0;
    private GameObject closestRizon;
    private float minDis = 10f;


    private void Start()
    {
        ROSConnection.GetOrCreateInstance().ImplementService<SpawnRizonVisualRequest, 
            SpawnRizonVisualResponse>(m_ServiceName, SpawnOneRizonVisual);
    }

    private SpawnRizonVisualResponse SpawnOneRizonVisual(SpawnRizonVisualRequest request)
    {
        CreateRizon(request.joint_state);
        SpawnRizonVisualResponse response = new SpawnRizonVisualResponse();
        response.success = true;

        return response;
    }

    private void CreateRizon(double[] joint_state)
    {
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/RizonVisual.prefab", (obj) =>
        {
            countRizon++;
            GameObject rizonVisual = GameObject.Instantiate(obj, new Vector3(0f, 0f, 0f), new Quaternion(0f, 0f, 0f, 1f));
            rizonVisual.name = "RizonVisual" + countRizon.ToString();
            rizonVisual.GetComponent<JointController>()._jointSource = "rizon_visual";
            float[] tempJoint = rizonVisual.GetComponent<JointController>()._jointState;
            for (int i = 0; i < tempJoint.Length; i++)
            {
                tempJoint[i] = (float)joint_state[i];
            }
            rizonList.Add(rizonVisual);
        });
    }

    private void Update()
    {
        float[] rizonJointState = GetComponent<JointController>()._jointState;
        DestroyRizon(rizonJointState);
        ChangeTransparency(rizonJointState);
    }

    private void ChangeTransparency(float[] rizonJointState)
    {
        if (closestRizon != null)
        {
            List<MeshRenderer> _allRenderer = new List<MeshRenderer>();
            Transform[] _transforms = closestRizon.GetComponentsInChildren<Transform>();
            foreach (Transform t in _transforms)
            {
                if (t.name == "default")
                {
                    _allRenderer.Add(t.gameObject.GetComponent<MeshRenderer>());
                }
            }
            float alpha = Mathf.Lerp(0f, 100f / 255f, minDis / 2f);
            StartCoroutine(FadeFade(_allRenderer, alpha));
        }
    }

    IEnumerator FadeFade(List<MeshRenderer> allRenderer, float alpha)
    {
        foreach (MeshRenderer m in allRenderer)
        {
            Color color = m.material.color;
            color.a = alpha;
            m.material.color = color;
        }
        yield return null;
    }

    private void DestroyRizon(float[] rizonJointState)
    {
        List<GameObject> toBeDestrory = new List<GameObject>();
        minDis = 10f;
        foreach (GameObject rizonVisual in rizonList)
        {
            float[] visualJointState = rizonVisual.GetComponent<JointController>()._jointState;
            float dis = 0f;
            for (int i = 0; i < visualJointState.Length; i++)
            {
                dis += Mathf.Abs(rizonJointState[i] - visualJointState[i]);
            }
            if (dis < minDis)
            {
                minDis = dis;
                closestRizon = rizonVisual;
            }
            if (dis < 0.1)
            {
                toBeDestrory.Add(rizonVisual);
            }
        }
        foreach (GameObject obj in toBeDestrory)
        {
            if (rizonList.Contains(obj))
            {
                rizonList.Remove(obj);
                Destroy(obj);
            }
        }
    }
}
