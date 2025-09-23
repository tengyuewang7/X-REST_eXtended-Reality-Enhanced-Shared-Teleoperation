using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class HanoiVisualizationGuidance : MonoBehaviour
{
    private HanoiGroup hanoiInfo; 
    private Vector3[] basePositions; 
    private GameObject[] hanois; 
    private uint[] hanoiStates; 
    private float[] baseHeight; 
    private bool[] isGrab; 

    private bool isMoving = false;
    private Material[] visualMaterials = new Material[3];
    private GameObject visualHanoi;
    /*
    private List<uint[]> stepList = new List<uint[]>
    {
        new uint[3] { 0, 0, 0 }, new uint[3] { 2, 0, 0 }, new uint[3] { 2, 1, 0 },
        new uint[3] { 1, 1, 0 }, new uint[3] { 1, 1, 2 }, new uint[3] { 0, 1, 2 },
        new uint[3] { 0, 2, 2 }, new uint[3] { 2, 2, 2 }
    }; 
    private List<uint[]> motionList = new List<uint[]>
    {
        new uint[3] { 0, 2, 0 }, new uint[3] { 1, 1, 0 }, new uint[3] { 0, 1, 1 },
        new uint[3] { 2, 2, 0 }, new uint[3] { 0, 0, 0 }, new uint[3] { 1, 2, 1 },
        new uint[3] { 0, 2, 2 }
    }; */
    private List<uint[]> stepList; 
    private List<uint[]> motionList; 
    private int stepIndex = 0;


    void Start()
    {
        // Solve the Tower of Hanoi: number of disk -> 3;
        // start position -> 0; end position -> 2; auxiliary position -> 1 
        (stepList, motionList) = SolveHanoi.GetList(3, 0, 2, 1); 
        // TODO: change position here 

        hanoiInfo = GetComponent<HanoiGroup>();
        basePositions = hanoiInfo.basePositions;
        hanois = hanoiInfo.hanois;
        hanoiStates = hanoiInfo.hanoiStates;
        baseHeight = hanoiInfo.baseHeight;
        isGrab = hanoiInfo.isGrab;


        string path = "Assets/Prefabs/Material"; 
        visualMaterials[0] = AssetDatabase.LoadAssetAtPath<Material>(path + "/WhiteTransparency.mat");
        visualMaterials[1] = AssetDatabase.LoadAssetAtPath<Material>(path + "/BlueTransparency.mat"); 
        visualMaterials[2] = AssetDatabase.LoadAssetAtPath<Material>(path + "/RedTransparency.mat");

        visualHanoi = Instantiate(hanois[0]);
        visualHanoi.transform.parent = transform;
        visualHanoi.name = "HanoiVisualGuidance"; 
        visualHanoi.GetComponent<Rigidbody>().useGravity = false;
        visualHanoi.GetComponent<Rigidbody>().collisionDetectionMode = CollisionDetectionMode.Discrete;
        visualHanoi.GetComponent<Rigidbody>().isKinematic = true;
        visualHanoi.GetComponent<BoxCollider>().enabled = false;
    }


    private IEnumerator VisualizationHanoiMove() 
    {
        if (isGrab[0] || isGrab[1] || isGrab[2] || stepIndex == stepList.Count - 1)
        {
            yield break;
        }

        isMoving = true;

        uint hanoiId = motionList[stepIndex][0];
        uint positionId = motionList[stepIndex][1];
        uint heightId = motionList[stepIndex][2];

        visualHanoi.transform.position = new Vector3(hanois[hanoiId].transform.position.x,
            hanois[hanoiId].transform.position.y, hanois[hanoiId].transform.position.z);
        visualHanoi.transform.rotation = hanois[hanoiId].transform.rotation;
        visualHanoi.transform.localScale = hanois[hanoiId].transform.localScale;
        visualHanoi.GetComponent<MeshRenderer>().material = visualMaterials[hanoiId];

        Vector3 targetPosition = new Vector3(basePositions[positionId].x, baseHeight[heightId], basePositions[positionId].z);
        Vector3 startPosition = targetPosition + new Vector3(0f, 0.08f, 0f);
        yield return StartCoroutine(MoveVisualHanoi(startPosition, targetPosition));


        isMoving = false;

        yield return null;
    }

    private IEnumerator MoveVisualHanoi(Vector3 startPosition, Vector3 targetPosition)
    {
        visualHanoi.GetComponent<MeshRenderer>().enabled = true;
        visualHanoi.transform.position = startPosition;
        float startTime = Time.time;
        float lerpDuration = 2f;
        float lerpTimer = 0f;
        while (lerpTimer <= lerpDuration)
        {
            if (isGrab[0] || isGrab[1] || isGrab[2])
            {
                visualHanoi.GetComponent<MeshRenderer>().enabled = false;
                yield break; 
            }
            lerpTimer = Time.time - startTime;

            float t = Mathf.Clamp01(lerpTimer / lerpDuration);
            visualHanoi.transform.position = Vector3.Lerp(startPosition, targetPosition, t);
            visualHanoi.transform.rotation = new Quaternion(0f, 0f, 0f, 1f);
            yield return new WaitForSeconds(0.0001f);
        }

        yield return new WaitForSeconds(1f);
    }

    private IEnumerator checkStep()
    {
        for (int i = 0; i < stepList.Count; i++)
        {
            if (hanoiStates[0] == stepList[i][0] && hanoiStates[1] == stepList[i][1] && hanoiStates[2] == stepList[i][2])
            {
                stepIndex = i;
                yield break;
            }
        }
        stepIndex = stepList.Count - 1;
    }

    void NextStep()
    {
        if (isMoving == true)
        {
            ;
        }
        else
        {
            StartCoroutine(checkStep());
            StartCoroutine(VisualizationHanoiMove());
        }
    }

    void Update()
    {
        NextStep();

    }
}
