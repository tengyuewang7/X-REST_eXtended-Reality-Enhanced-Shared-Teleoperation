using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawBaseLine : MonoBehaviour
{
    private HanoiGroup hanoiInfo;
    private Vector3[] basePositions;
    void Start()
    {
        hanoiInfo = GetComponent<HanoiGroup>();

        basePositions = hanoiInfo.basePositions;

        // ����ÿ���߶εĶ���
        Vector3 height = new Vector3(0f, 0.1f, 0f);
        Vector3[][] linePoints = new Vector3[][]
        {
            new Vector3[] { basePositions[0],  basePositions[0] + height },
            new Vector3[] { basePositions[1],  basePositions[1] + height },
            new Vector3[] { basePositions[2],  basePositions[2] + height }
        };

        DrawLines(linePoints);
    }

    void DrawLines(Vector3[][] linePoints)
    {
        for (int i = 0; i < linePoints.Length; i++)
        {
            GameObject lineObj = new GameObject("Line" + i);
            lineObj.transform.parent = this.transform;

            LineRenderer lineRenderer = lineObj.AddComponent<LineRenderer>();

            // ���������Ĳ��ʡ���ɫ�Ϳ��
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.material.color = Color.red;
            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.red;
            lineRenderer.startWidth = 0.002f;
            lineRenderer.endWidth = 0.002f;

            // �����߶εĶ�������
            lineRenderer.positionCount = linePoints[i].Length;

            // �����߶ε�λ��
            lineRenderer.SetPositions(linePoints[i]);
        }
    }
}
