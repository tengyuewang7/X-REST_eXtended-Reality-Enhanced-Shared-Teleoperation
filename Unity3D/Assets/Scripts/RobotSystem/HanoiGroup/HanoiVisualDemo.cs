using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HanoiVisualDemo : MonoBehaviour
{
    private HanoiGroup hanoiInfo;
    private Vector3[] basePositions;
    private GameObject[] hanois;
    private uint[] hanoiStates;
    private float[] baseHeight;

    private bool isMoving = false;

    void Start()
    {
        hanoiInfo = GetComponent<HanoiGroup>();
        basePositions = hanoiInfo.basePositions;
        hanois = hanoiInfo.hanois;

        hanoiStates = hanoiInfo.hanoiStates;
        baseHeight = hanoiInfo.baseHeight;

    }


    private IEnumerator MoveHanoiBlock(uint hanoiId, uint positionId, uint heightId)
    {
        isMoving = true;
        hanois[hanoiId].GetComponent<Rigidbody>().useGravity = false;

        Vector3 startPosition = hanois[hanoiId].transform.position;

        Vector3 targetposition = startPosition + new Vector3(0f, 0.08f, 0f);
        yield return StartCoroutine(MoveBlock(hanoiId, targetposition));
        targetposition.x = basePositions[positionId].x;
        targetposition.z = basePositions[positionId].z;
        yield return StartCoroutine(MoveBlock(hanoiId, targetposition));
        targetposition.y = baseHeight[heightId];
        yield return StartCoroutine(MoveBlock(hanoiId, targetposition));

        isMoving = false;
        hanois[hanoiId].GetComponent<Rigidbody>().useGravity = true;

        yield return new WaitForSeconds(0.01f);
    }

    private IEnumerator MoveBlock(uint hanoiId, Vector3 targetPosition)
    {
        Vector3 startPoint = hanois[hanoiId].transform.position;
        float startTime = Time.time;
        float lerpDuration = 2f;
        float lerpTimer = 0f;
        while (lerpTimer <= lerpDuration)
        {
            //lerpTimer += Time.deltaTime;
            lerpTimer = Time.time - startTime;

            float t = Mathf.Clamp01(lerpTimer / lerpDuration);
            hanois[hanoiId].transform.position = Vector3.Lerp(startPoint, targetPosition, t);
            hanois[hanoiId].transform.rotation = new Quaternion(0f, 0f, 0f, 1f);
            yield return new WaitForSeconds(0.001f);
        }
    }

    void NextStep()
    {
        if (isMoving == true)
        {
            ;
        }
        else
        {
            if (hanoiStates[0] == 0 && hanoiStates[1] == 0 && hanoiStates[2] == 0)
            {
                StartCoroutine(MoveHanoiBlock(0, 2, 0));
            }
            else if (hanoiStates[0] == 2 && hanoiStates[1] == 0 && hanoiStates[2] == 0)
            {
                StartCoroutine(MoveHanoiBlock(1, 1, 0));
            }
            else if (hanoiStates[0] == 2 && hanoiStates[1] == 1 && hanoiStates[2] == 0)
            {
                StartCoroutine(MoveHanoiBlock(0, 1, 1));

            }
            else if (hanoiStates[0] == 1 && hanoiStates[1] == 1 && hanoiStates[2] == 0)
            {
                StartCoroutine(MoveHanoiBlock(2, 2, 0));

            }
            else if (hanoiStates[0] == 1 && hanoiStates[1] == 1 && hanoiStates[2] == 2)
            {
                StartCoroutine(MoveHanoiBlock(0, 0, 0));
            }
            else if (hanoiStates[0] == 0 && hanoiStates[1] == 1 && hanoiStates[2] == 2)
            {
                StartCoroutine(MoveHanoiBlock(1, 2, 1));

            }
            else if (hanoiStates[0] == 0 && hanoiStates[1] == 2 && hanoiStates[2] == 2)
            {
                StartCoroutine(MoveHanoiBlock(0, 2, 2));

            }
            else if (hanoiStates[0] == 2 && hanoiStates[1] == 2 && hanoiStates[2] == 2)
            {
                // Debug.Log("Done");
            }
        }
    }

    void Update()
    {
        NextStep();
    }
}
