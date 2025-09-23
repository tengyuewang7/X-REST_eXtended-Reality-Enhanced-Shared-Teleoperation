using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FailureDetection : MonoBehaviour
{
    private HanoiGroup hanoiInfo;
    private Vector3[] basePositions;
    private GameObject[] hanois;
    private uint[] hanoiStates;
    private bool[] isGrab; 

    void Start()
    {
        hanoiInfo = GetComponent<HanoiGroup>();
        basePositions = hanoiInfo.basePositions;
        hanois = hanoiInfo.hanois;
        hanoiStates = hanoiInfo.hanoiStates;
        isGrab = hanoiInfo.isGrab;
    }

    private IEnumerator Rule1()
    {
        // Rule 1: Only one disk can be moved at a time. 
        for (int i = 0; i < isGrab.Length; i++)
        {
            if (isGrab[i])
            {
                for (int h = 0; h < hanois.Length; h++)
                {
                    if (h != i)
                    {
                        hanois[h].GetComponent<ObjectManipulator>().enabled = false;
                    }
                }
            }
        }
        if (!isGrab[0] && !isGrab[1] && !isGrab[2])
        {
            for (int h = 0; h < hanois.Length; h++)
            {
                hanois[h].GetComponent<ObjectManipulator>().enabled = true;
            }
        }
        yield return null; 
    }

    private IEnumerator Rule2()
    {
        // Rule 2: Each move consists of taking the upper disk from
        // one of the stacks and placing it on top of another stack
        // i.e. a disk can only be moved if it is the uppermost disk on a stack.
        bool[][] baseStates = new bool[3][]; //  which hanoi is at each base position
        baseStates[0] = new bool[hanoiStates.Length];
        baseStates[1] = new bool[hanoiStates.Length];
        baseStates[2] = new bool[hanoiStates.Length];
        for (int i = 0; i < 3; i++) // 3 base positions 
        {
            bool nothingOnTop = true;
            for (int j = 0; j < hanoiStates.Length; j++)
            {
                if (hanoiStates[j] == i)
                {
                    if (nothingOnTop)
                    {
                        hanois[j].GetComponent<ObjectManipulator>().enabled = true;
                        nothingOnTop = false;
                    }
                    else
                    {
                        hanois[j].GetComponent<ObjectManipulator>().enabled = false;
                    }
                }

            }
        }
        yield return null;
    }

    private IEnumerator Rule3()
    {
        // Rule 3: No disk may be placed on top of a smaller disk. 
        // Has an exception at the first step when not place HanoiA onto a peg. 
        if (!isGrab[0] && !isGrab[1] && !isGrab[2])
        {
            for (int i = 0; i < 3; i++)
            {
                float maxHeight = 0;
                int cnt = 0;
                for (int j = 0; j < hanoiStates.Length; j++)
                {
                    if (hanoiStates[j] == i && hanois[j].transform.position.y > maxHeight + 0.001)
                    {
                        cnt++;
                        maxHeight = hanois[j].transform.position.y;
                    }
                    if (cnt > 1) 
                    {
                        // Debug.LogWarning("Do not place a larger disk on top of a smaller one!!!");
                        StartCoroutine(hanoiInfo.InitGame());
                    }
                }
            }
        }
        yield return null;
    }

    private IEnumerator Rule4()
    {
        // Rule 4: 
        if (!isGrab[0] && !isGrab[1] && !isGrab[2])
        {
            foreach (GameObject h in hanois)
            {
                float minDis = 10f; 
                foreach (Vector3 v3 in basePositions)
                {
                    float dis = (h.transform.position.x - v3.x) *
                        (h.transform.position.x - v3.x) +
                        (h.transform.position.z - v3.z) *
                        (h.transform.position.z - v3.z); 
                    if (dis < minDis)
                    {
                        minDis = dis; 
                    }
                }
                if (minDis > 0.0001)
                {
                    // Debug.LogWarning("Place the disk onto a peg!!!");
                    StartCoroutine(hanoiInfo.InitGame());
                }
            }
        }
        yield return null;
    }

    void Update()
    {
        StartCoroutine(Rule1());
        StartCoroutine(Rule2());
        // StartCoroutine(Rule3()); 
        // StartCoroutine(Rule4());

    }
}
