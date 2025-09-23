using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class SuctionCheck : MonoBehaviour
{
    private GameObject ros2 = null;
    private GameObject checkPoint = null;
    private HanoiGroup hanoiGroup = null; 
    private RosCommunication rosCommunication = null; 
    // Start is called before the first frame update
    void Start()
    {
        ros2 = GameObject.Find("RosCommunication");
        rosCommunication = ros2.GetComponent<RosCommunication>();
        checkPoint = GameObject.Find("Rizon/world/base_link/link_1/link_2/link_3/link_4/link_5/link_6/link_7/flange/sucker/sucker_disc/suction_check_point");
        hanoiGroup = GameObject.Find("HanoiGroup").GetComponent<HanoiGroup>();
    }

    // Update is called once per frame
    void Update()
    {
        float minDistance = 10f;
        int index = 10;
        for (int i = 0; i < 3; i++)
        {
            float distance = Vector3.Distance(checkPoint.transform.position, hanoiGroup.hanois[i].transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                index = i;
            }
        }

        if (rosCommunication.is_sucked)
        {
            hanoiGroup.hanois[index].GetComponent<Rigidbody>().useGravity = false;
            hanoiGroup.hanois[index].transform.position = checkPoint.transform.position;
            hanoiGroup.hanois[index].transform.rotation = checkPoint.transform.rotation;
            if (index == 0)
            {
                hanoiGroup.IsGrabA();
            }
            else if (index == 1)
            {
                hanoiGroup.IsGrabB();
            }
            else if (index == 2)
            {
                hanoiGroup.IsGrabC(); 
            }
        }
        else
        {
            for (int i = 0; i < 3; i++) {
                hanoiGroup.hanois[i].GetComponent<Rigidbody>().useGravity = true;
            }
            hanoiGroup.IsReleaseA(); 
            hanoiGroup.IsReleaseB();
            hanoiGroup.IsReleaseC(); 
        }
    }
}
