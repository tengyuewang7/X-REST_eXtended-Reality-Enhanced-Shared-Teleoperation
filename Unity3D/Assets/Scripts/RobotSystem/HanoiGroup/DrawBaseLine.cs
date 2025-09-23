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

        // 定义每条线段的顶点
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

            // 设置线条的材质、颜色和宽度
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.material.color = Color.red;
            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.red;
            lineRenderer.startWidth = 0.002f;
            lineRenderer.endWidth = 0.002f;

            // 设置线段的顶点数量
            lineRenderer.positionCount = linePoints[i].Length;

            // 设置线段的位置
            lineRenderer.SetPositions(linePoints[i]);
        }
    }
}
