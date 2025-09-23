using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class MeshVisualization : MonoBehaviour
{
    //public GameObject meshObject;
    private string topicName = "/mesh";
    private ROSConnection ros;

    private bool flag;

    private GameObject meshObject;

    private List<Vector3> points = new List<Vector3>(); 

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PointCloudMsg>(topicName, ReceivePointCloud);

        flag = false;

        meshObject = new GameObject("themesh"); 

        meshObject.AddComponent<MeshRenderer>(); 
        meshObject.AddComponent<MeshFilter>(); 



        //Mesh mesh = meshObject.GetComponent<MeshCollider>().sharedMesh;
        //Vector3[] vertices = mesh.vertices;
        //foreach (Vector3 vertex in vertices)
        //{
        //    Vector3 worldPos = transform.TransformPoint(vertex);
        //    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //    sphere.transform.position = worldPos; 
        //    sphere.transform.localScale = Vector3.one * 0.01f; 
        //}
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void ReceivePointCloud(PointCloudMsg msg)
    {
        //if (flag)
        //{
        //    return; 
        //}
        //flag = true; 

        points.Clear(); 

        var pointsArray = msg.points; 

        for (int i = 0; i < pointsArray.Length; i++)
        {
            var p = pointsArray[i];

            // ROS坐标系与Unity坐标系可能不一致，按需变换
            Vector3 position = p.From<FLU>();

            points.Add(position); 

            //GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            //sphere.transform.position = position;
            //sphere.transform.localScale = Vector3.one * 0.01f;

        }

        meshObject.GetComponent<MeshFilter>().mesh.Clear();
        meshObject.GetComponent<MeshFilter>().mesh.vertices = points.ToArray();

        // 创建索引数组
        int[] indices = new int[pointsArray.Length];
        for (int i = 0; i < pointsArray.Length; i++)
        {
            indices[i] = i;
        }
        meshObject.GetComponent<MeshFilter>().mesh.SetIndices(indices, MeshTopology.Points, 0);

        MaterialPropertyBlock props = new MaterialPropertyBlock();
        props.SetFloat("_PointSize", 0.05f);
        meshObject.GetComponent<MeshRenderer>().SetPropertyBlock(props);

        //foreach (Vector3 vertex in pointsArray.Length)
        //{
        //    Vector3 worldPos = transform.TransformPoint(vertex);
        //    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //    sphere.transform.position = worldPos;
        //    sphere.transform.localScale = Vector3.one * 0.01f;
        //}
    }
}
