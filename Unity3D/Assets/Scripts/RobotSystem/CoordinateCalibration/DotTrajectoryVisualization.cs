using Cysharp.Threading.Tasks;
using JarvenFramework.ResModule;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class DotTrajectoryVisualization : MonoBehaviour
{

    Vector3 lastPos; 

    Queue<GameObject> trajQueue = new Queue<GameObject>();
    GameObject dotTraj; 

    private long dotID = 0L;

    // Start is called before the first frame update
    void Start()
    {
        dotTraj = new GameObject("DotTraj");
        lastPos = transform.position; 
    }

    private async UniTaskVoid CreateDot()
    {
        while (true)
        {
            if ((transform.position - lastPos).magnitude > 0.003)
            {
                ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Dot.prefab", (obj) =>
                {
                    GameObject dot = GameObject.Instantiate(obj, transform.position, new Quaternion(0f, 0f, 0f, 1f));
                    dot.name = "dot" + dotID;
                    dotID++;
                    Color c = Color.red;
                    c.a = 1f;
                    dot.GetComponent<MeshRenderer>().material.color = c;
                    trajQueue.Enqueue(dot);
                    dot.transform.SetParent(dotTraj.transform); 
                });
                lastPos = transform.position;
            }
            await UniTask.DelayFrame(1); 
        }

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        CreateDot().Forget(); 

        StartCoroutine(ManageDots());
    }


    IEnumerator ManageDots()
    {
        int countExpired = 0;
        foreach (var dot in trajQueue)
        {
            Color c = dot.GetComponent<MeshRenderer>().material.color;

            c.a -= 0.01f;
            dot.GetComponent<MeshRenderer>().material.color = c;
            if (c.a < 0.01f)
            {
                countExpired++;
            }
        }
        for (int i = 0; i < countExpired; i++)
        {
            Destroy(trajQueue.Dequeue());
        }
        yield return null; 
    }
}
