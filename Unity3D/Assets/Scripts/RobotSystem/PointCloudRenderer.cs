using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using static Unity.Robotics.UrdfImporter.Link.Geometry;
using System.IO;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Microsoft.MixedReality.Toolkit.Utilities;

public class PointCloudRenderer : MonoBehaviour
{
    [HideInInspector]
    public List<Vector3> pointCloudPositionList = new List<Vector3>();
    [HideInInspector]
    public List<Color> pointCloudColorList = new List<Color>(); 
    private List<GameObject> points = new List<GameObject>();

    private UnityEngine.Mesh mesh;
    private UnityEngine.MeshFilter meshFilter;
    private UnityEngine.MeshRenderer meshRenderer;

    void Start()
    {
        mesh = new UnityEngine.Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 // ֧�ָ��ඥ��
        };

        // ��ȡ�����MeshFilter��MeshRenderer���
        meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        // ʹ�ü򵥵���ɫ��
        meshRenderer.material = Resources.Load<Material>("Assets/Prefabs/Material/MyPointCloudMaterial.mat");


        //meshRenderer.enabled = false;

        LoadPointsFromFile();

        //ExportPointsToTxt(pointCloudPositionList, "Assets/Scripts/RobotSystem/RizonRobot/filtered_point_cloud.txt");

        UpdateOnce();
    }

    void LoadPointsFromFile()
    {
        string filePath = "Assets/Scripts/RobotSystem/RizonRobot/filtered_point_cloud.txt";
        int cnt = 0;

        // Check if file exists
        if (File.Exists(filePath))
        {
            string[] lines = File.ReadAllLines(filePath);

            foreach (string line in lines)
            {
                string[] coordinates = line.Split(',');
                float x = float.Parse(coordinates[0].Trim());
                float y = float.Parse(coordinates[1].Trim());
                float z = float.Parse(coordinates[2].Trim());
                
                // Convert from ROS FLU to Unity
                PointMsg rosPoint = new PointMsg(x, y, z);
                Vector3 point = rosPoint.From<FLU>();

                // filtered
                pointCloudPositionList.Add(point);
                pointCloudColorList.Add(Color.red);
                cnt++; 

                //// non-filtered
                //if (point.y > 0.05f && point.z < 0.6f)
                //{
                //    point.y = point.y - 0.02f;
                //    //Debug.Log("Loaded point: " + point);
                //    pointCloudPositionList.Add(point);
                //    pointCloudColorList.Add(Color.red);
                //    cnt++;
                //}
            }
        }
        else
        {
            Debug.LogError("File not found: " + filePath);
        }
        Debug.Log("Load " +  cnt + " point cloud.");
    }

    void ExportPointsToTxt(List<Vector3> points, string filePath)
    {
        using (StreamWriter writer = new StreamWriter(filePath))
        {
            foreach (Vector3 point in points)
            {
                PointMsg rosPoint = point.To<FLU>();

                // ��ÿ�� Vector3 д���ı��ļ�����ʽΪ "x, y, z"
                writer.WriteLine($"{rosPoint.x}, {rosPoint.y}, {rosPoint.z}");
            }
        }

        Debug.Log($"Exported {points.Count} points to {filePath}");
    }

    Vector3 ParsePoint(string line)
    {
        string[] coordinates = line.Split(',');
        float x = float.Parse(coordinates[0].Trim());
        float y = float.Parse(coordinates[1].Trim());
        float z = float.Parse(coordinates[2].Trim());

        // Convert from ROS FLU to Unity
        PointMsg rosPoint = new PointMsg(x, y, z);
        Vector3 res = rosPoint.From<FLU>();

        return res;
    }

    void UpdateOnce()
    {
        // ����Mesh�Ķ������ɫ
        mesh.vertices = pointCloudPositionList.ToArray();
        mesh.colors = pointCloudColorList.ToArray();

        // ����һ���򵥵ĵ��б�
        int[] indices = new int[pointCloudPositionList.Count];
        for (int i = 0; i < indices.Length; i++)
        {
            indices[i] = i;
        }

        // ����Mesh�ĵ�
        mesh.SetIndices(indices, MeshTopology.Points, 0);

        // ��Mesh�����MeshFilter
        meshFilter.mesh = mesh;

    }

    //void Update()
    //{

    //    // ȷ�������ݿ���
    //    if (pointCloudPositionList.Count == 0 || pointCloudColorList.Count == 0)
    //    {
    //        return;
    //    }

    //    // ����Mesh�Ķ������ɫ
    //    mesh.vertices = pointCloudPositionList.ToArray();
    //    mesh.colors = pointCloudColorList.ToArray();

    //    // ����һ���򵥵ĵ��б�
    //    int[] indices = new int[pointCloudPositionList.Count];
    //    for (int i = 0; i < indices.Length; i++)
    //    {
    //        indices[i] = i;
    //    }

    //    // ����Mesh�ĵ�
    //    mesh.SetIndices(indices, MeshTopology.Points, 0);

    //    // ��Mesh�����MeshFilter
    //    meshFilter.mesh = mesh;

    //}

    public void SetVisibility()
    {
        meshRenderer.enabled = !meshRenderer.enabled;
    }

}
