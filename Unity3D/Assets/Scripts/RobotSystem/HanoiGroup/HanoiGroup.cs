using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HanoiGroup : MonoBehaviour
{
    // The setup has three base states: 0, 1, 2; three hanoi blocks: A, B, C.  
    // The start states of three hanoi blocks are 0, and the target states are all . 
    // Switch the BasePosition when states are switched. 
    // This script is used to store information of Hanoi group. 

    public Vector3[] basePositions = new Vector3[3]; // three fixed base position 
    public GameObject[] hanois = new GameObject[3]; // Hanois: {A, B, C}  
    public GameObject[] hanoiBases = new GameObject[3]; // Three bases 
    public float[] baseHeight = new float[3]; // At each base position, 3 height 
    public uint[] hanoiStates = new uint[3];
    public bool[] isGrab = new bool[3];  // {A, B, C}. Tell whether and which hanoi block is grabbed in MR mode. 
    public Vector3[] staticHanoiPosition; 

    void Start()
    {
        basePositions[0] = new Vector3(-0.230f, 0f, 0.664f);
        basePositions[1] = new Vector3(0.221f, 0f, 0.709f);
        basePositions[2] = new Vector3(0.352f, 0f, 0.339f);

        baseHeight = new float[3] { 0.01f, 0.03f, 0.05f };

        hanoiStates = new uint[3] { 0, 0, 0 };

        isGrab = new bool[3] { false, false, false };  // will be updated 

        Transform[] childTransform = GetComponentsInChildren<Transform>();
        foreach (Transform t in childTransform)
        {
            if (t.name == "HanoiA")
            {
                hanois[0] = t.gameObject;
            }
            else if (t.name == "HanoiB")
            {
                hanois[1] = t.gameObject;
            }
            else if (t.name == "HanoiC")
            {
                hanois[2] = t.gameObject;
            }
            else if (t.name == "Base1")
            {
                hanoiBases[0] = t.gameObject; 
            }
            else if (t.name == "Base2")
            {
                hanoiBases[1] = t.gameObject;
            }
            else if (t.name == "Base3")
            {
                hanoiBases[2] = t.gameObject;
            }
        }

        hanoiBases[0].transform.position = basePositions[0] + new Vector3(0f, -0.01f, 0f);
        hanoiBases[1].transform.position = basePositions[1] + new Vector3(0f, -0.01f, 0f);
        hanoiBases[2].transform.position = basePositions[2] + new Vector3(0f, -0.01f, 0f);

        hanois[2].transform.position = basePositions[0] + new Vector3(0f, 0.01f, 0f); // Move the lower one first. 
        hanois[1].transform.position = basePositions[0] + new Vector3(0f, 0.03f, 0f);
        hanois[0].transform.position = basePositions[0] + new Vector3(0f, 0.05f, 0f);

        staticHanoiPosition = new Vector3[3];
        staticHanoiPosition[0] = hanois[0].transform.position;
        staticHanoiPosition[1] = hanois[1].transform.position;
        staticHanoiPosition[2] = hanois[2].transform.position;
    }

    private void Update()
    {
        UpdateHanoiStates(); 
        UpdateStaticHanoiPositions();
        UpdateHanoiOrientation();
    }

    private void UpdateHanoiStates()
    {
        for (uint h = 0; h < hanois.Length; h++)
        {
            for (uint i = 0; i < basePositions.Length; i++)
            {
                double dis = (hanois[h].transform.position.x - basePositions[i].x) *
                    (hanois[h].transform.position.x - basePositions[i].x) +
                    (hanois[h].transform.position.z - basePositions[i].z) *
                    (hanois[h].transform.position.z - basePositions[i].z);
                if (dis < 0.0002 && !isGrab[0] && !isGrab[1] && !isGrab[2])
                {
                    hanoiStates[h] = i;
                }
            }
        }
    }

    private void UpdateStaticHanoiPositions()
    {
        for (uint h = 0; h < hanois.Length; h++)
        {
            double dis = (staticHanoiPosition[h] - hanois[h].transform.position).magnitude;
            if (isGrab[h] == false && dis > 0.0002)
            {
                staticHanoiPosition[h] = hanois[h].transform.position;
            }
        }
    }

    private void UpdateHanoiOrientation()
    {
        for (uint i = 0; i < hanois.Length; i++)
        {
            if (isGrab[i] == false)
            {
                hanois[i].transform.rotation = Quaternion.Euler(0f, 0f, 0f);
            }
        }
    }

    public void IsGrabA()
    {
        isGrab[0] = true;
    }
    public void IsReleaseA()
    {
        isGrab[0] = false; 
    }
    public void IsGrabB()
    {
        isGrab[1] = true;
    }
    public void IsReleaseB()
    {
        isGrab[1] = false;
    }
    public void IsGrabC()
    {
        isGrab[2] = true;
    }
    public void IsReleaseC()
    {
        isGrab[2] = false;
    }

    internal IEnumerator InitGame()
    {
        // Debug.Log("Restarting the game ..."); 
        while (hanoiStates[0] != 0 || hanoiStates[1] != 0 || hanoiStates[2] != 0)
        {
            hanois[2].transform.position = basePositions[0] + new Vector3(0f, baseHeight[0], 0f);
            hanois[2].transform.rotation = new Quaternion(0, 0, 0, 1);
            hanois[1].transform.position = basePositions[0] + new Vector3(0f, baseHeight[1], 0f);
            hanois[1].transform.rotation = new Quaternion(0, 0, 0, 1);
            hanois[0].transform.position = basePositions[0] + new Vector3(0f, baseHeight[2], 0f);
            hanois[0].transform.rotation = new Quaternion(0, 0, 0, 1);
            yield return new WaitForSeconds(0.01f);
        }
    }
}
