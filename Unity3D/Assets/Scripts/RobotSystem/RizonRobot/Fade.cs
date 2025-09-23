using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Net;
using Microsoft.MixedReality.Toolkit.Utilities;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Flexiv;
using static Unity.Robotics.UrdfImporter.Link.Geometry;

public class Fade : MonoBehaviour
{
    private LineRenderer lineRenderer;

    private GameObject meshPoint;
    private GameObject obstaclePoint;

    public Material sphereMaterial;
    public Material lineMaterial; 


    private ROSConnection ros;

    void Start()
    {

        ros = ROSConnection.GetOrCreateInstance();

        ROSConnection.GetOrCreateInstance().Subscribe<ClosestPointMsg>("closest_point", ReceiveCallback);

        meshPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        meshPoint.transform.localScale = Vector3.one * 0.01f;
        meshPoint.GetComponent<MeshRenderer>().material = sphereMaterial;   
        meshPoint.transform.parent = transform; 
        meshPoint.GetComponent<SphereCollider>().enabled = false; 

        obstaclePoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        obstaclePoint.transform.localScale = Vector3.one * 0.01f;
        obstaclePoint.GetComponent<MeshRenderer>().material = sphereMaterial;
        obstaclePoint.transform.parent = transform;
        obstaclePoint.GetComponent<SphereCollider>().enabled = false;

        lineRenderer = gameObject.AddComponent<LineRenderer>();

        lineRenderer.material = lineMaterial; 

        //lineRenderer.startColor = Color.red;
        //lineRenderer.endColor = Color.red;

        lineRenderer.startWidth = 0.003f;
        lineRenderer.endWidth = 0.003f;

        lineRenderer.positionCount = 2;

        sphereMaterial.color = new Color(sphereMaterial.color.r, sphereMaterial.color.g, sphereMaterial.color.b, 1.0f);
        lineMaterial.color = new Color(lineMaterial.color.r, lineMaterial.color.g, lineMaterial.color.b, 1.0f);
    }

    void Update()
    {

    }

    void ReceiveCallback(ClosestPointMsg msg)
    {
        meshPoint.transform.position = msg.mesh_point.From<FLU>();
        obstaclePoint.transform.position = msg.obstacle_point.From<FLU>(); 
        //if (Vector3.Distance(meshPoint.transform.position, obstaclePoint.transform.position) < 0.2f)
        //{ 
            //sphereMaterial.color = new Color(sphereMaterial.color.r, sphereMaterial.color.g, sphereMaterial.color.b, 1.0f);
            //lineMaterial.color = new Color(lineMaterial.color.r, lineMaterial.color.g, lineMaterial.color.b, 1.0f);
            lineRenderer.SetPosition(0, meshPoint.transform.position);
            lineRenderer.SetPosition(1, obstaclePoint.transform.position);
        //} 
        //else 
        //{
        //    sphereMaterial.color = new Color(sphereMaterial.color.r, sphereMaterial.color.g, sphereMaterial.color.b, 0.0f);
        //    lineMaterial.color = new Color(lineMaterial.color.r, lineMaterial.color.g, lineMaterial.color.b, 0.0f);
        //}
    }

}
