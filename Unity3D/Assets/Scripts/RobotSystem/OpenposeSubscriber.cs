using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.OpenposeRos2;
using System.Collections.Generic;
using System;
using System.Collections;

public class OpenposeSubscriber : MonoBehaviour
{
    private string _topicName = "pose_keypoints_list_topic";
    private PoseKeyPointsListMsg _poseKeyPointsListMsgs;
    private List<PersonInfo> personList = new List<PersonInfo>();


    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PoseKeyPointsListMsg>(_topicName, ReceiveCallback);
    }
    private void ReceiveCallback(PoseKeyPointsListMsg msg)
    {
        _poseKeyPointsListMsgs = msg;
        StartCoroutine(UpdatePersonList());
    }

    bool IsKeyPointvalid(PoseKeyPointMsg point)
    {
        if (Mathf.Abs(point.x) + Mathf.Abs(point.y) + Mathf.Abs(point.z) > 0.1)
        {
            return true;
        }
        return false;  
    }

    private IEnumerator UpdatePersonList()
    {
        while (personList.Count < _poseKeyPointsListMsgs.pose_key_points_list.Length)
        {
            personList.Add(new PersonInfo("Person" + personList.Count));
        }
        while (personList.Count > _poseKeyPointsListMsgs.pose_key_points_list.Length)
        {
            PersonInfo lastObject = personList[personList.Count - 1];
            personList.RemoveAt(personList.Count - 1);
            Destroy(lastObject.person);
        }

        for (int i = 0; i < _poseKeyPointsListMsgs.pose_key_points_list.Length; i++)
        {
            PoseKeyPointsMsg personKeyPoints = _poseKeyPointsListMsgs.pose_key_points_list[i];
            PersonInfo person = personList[i];
            foreach (GameObject g in person.joints)
            {
                g.SetActive(false);
            }
            foreach (GameObject g in person.links)
            {
                g.SetActive(false);
            }
            for (int j = 0; j < person.connections.Count; j++)
            {
                Tuple<int, int> linkInfo = person.connections[j];
                PoseKeyPointMsg keyPoint1 = personKeyPoints.pose_key_points[linkInfo.Item1];
                PoseKeyPointMsg keyPoint2 = personKeyPoints.pose_key_points[linkInfo.Item2];

                // OpenCV coordinate to Unity coordinate 
                Vector3 v1 = new Vector3(keyPoint1.x / 1000f, -keyPoint1.y / 1000f, keyPoint1.z / 1000f);
                Vector3 v2 = new Vector3(keyPoint2.x / 1000f, -keyPoint2.y / 1000f, keyPoint2.z / 1000f);


                if (IsKeyPointvalid(keyPoint1) && IsKeyPointvalid(keyPoint2))
                {
                    person.joints[linkInfo.Item1].SetActive(true);
                    person.joints[linkInfo.Item2].SetActive(true);
                    person.links[j].SetActive(true);

                    person.joints[linkInfo.Item1].transform.position = v1;
                    person.joints[linkInfo.Item2].transform.position = v2;
                    Vector3 diff = v2 - v1;
                    Vector3 mid = (v1 + v2) / 2;
                    person.links[j].transform.localScale = new Vector3(person.links[j].transform.localScale.x, 
                        diff.magnitude / 2 - 0.05f, person.links[j].transform.localScale.z);
                    person.links[j].transform.position = mid;
                    Quaternion targetRotation = Quaternion.FromToRotation(person.links[j].transform.up, diff);
                    person.links[j].transform.rotation = targetRotation * person.links[j].transform.rotation;
                }
            }
        }
        yield return null;
    }
}


public class PersonInfo
{
    public String[] jointName = new string[25]
    {
        "Nose", "Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist",
        "MidHip", "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar",
        "LEar", "LBigToe", "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel",
    };

    public List<Tuple<int, int>> connections = new List<Tuple<int, int>>
    {
        Tuple.Create(0, 1),
        Tuple.Create(0, 15),
        Tuple.Create(0, 16),
        Tuple.Create(1, 2),
        Tuple.Create(1, 5),
        Tuple.Create(1, 8),
        Tuple.Create(2, 3),
        Tuple.Create(3, 4),
        Tuple.Create(5, 6),
        Tuple.Create(6, 7),
        Tuple.Create(8, 9),
        Tuple.Create(8, 12),
        Tuple.Create(9, 10),
        Tuple.Create(10, 11),
        Tuple.Create(11, 22),
        Tuple.Create(11, 24),
        Tuple.Create(12, 13),
        Tuple.Create(13, 14),
        Tuple.Create(14, 19),
        Tuple.Create(14, 21),
        Tuple.Create(15, 17),
        Tuple.Create(16, 18),
        Tuple.Create(19, 20),
        Tuple.Create(22, 23),
    };
    public GameObject person;
    public List<GameObject> joints;
    public List<GameObject> links;

    public PersonInfo(string name)
    {
        InitAPeroson(name);
    }
    private void InitAPeroson(string name)
    {
        person = new GameObject(name);
        joints = new List<GameObject>();
        foreach (string n in jointName)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = n;
            sphere.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
            joints.Add(sphere);
            sphere.transform.SetParent(person.transform);
            sphere.GetComponent<MeshRenderer>().material.color = Color.red;
        }
        links = new List<GameObject>();
        foreach (Tuple<int, int> linkInfo in connections)
        {
            GameObject capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            capsule.name = jointName[linkInfo.Item1] + "-" + jointName[linkInfo.Item2];
            capsule.transform.localScale = new Vector3(0.03f, 0.0f, 0.03f);
            links.Add(capsule);
            capsule.transform.SetParent(person.transform);
            capsule.GetComponent<MeshRenderer>().material.color = Color.green;
        }
    }
}
