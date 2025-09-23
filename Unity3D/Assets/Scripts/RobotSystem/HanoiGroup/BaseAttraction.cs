using UnityEngine;
using System.Collections.Generic;


public class BaseAttraction : MonoBehaviour
{
    private HanoiGroup hanoiInfo;
    private Vector3[] basePositions;
    private GameObject[] hanois;

    void Start()
    {
        hanoiInfo = GetComponent<HanoiGroup>();
        hanois = hanoiInfo.hanois;
        basePositions = hanoiInfo.basePositions;
    }

    void Update()
    {
        for (int h = 0; h < hanois.Length; h++)
        {
            if (hanoiInfo.isGrab[h] == false)
            {
                double minDis = 100f;
                uint minPos = 0;
                for (uint i = 0; i < basePositions.Length; i++)
                {
                    Vector3 v3 = basePositions[i];
                    double dis = (hanois[h].transform.position.x - v3.x) *
                        (hanois[h].transform.position.x - v3.x) +
                        (hanois[h].transform.position.z - v3.z) *
                        (hanois[h].transform.position.z - v3.z);

                    if (dis < minDis)
                    {
                        minDis = dis;
                        minPos = i;
                    }
                }
                if (minDis < 0.0001)
                {
                    hanois[h].transform.position = new Vector3(basePositions[minPos].x,
                        hanois[h].transform.position.y, basePositions[minPos].z);
                }
            }
        }
    }
}
