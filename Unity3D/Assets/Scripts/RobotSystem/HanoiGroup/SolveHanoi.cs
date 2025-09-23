using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SolveHanoi : MonoBehaviour
{
    static private char[] diskName; // Start from "A" which is samllest in size. { "A", "B", "C", ... } 
    // The location of each hanoi disk at each step.
    // e.g. [2, 0, 1]: diskA at position 2, diskB at position 0, diskC at position 1. 
    static private List<uint[]> stepList = new List<uint[]>();
    // The best motion at the corresponding step in stepList.
    // e.g. [0, 2, 1]: move diskA (index is 0) to position 2 at height 1 (position 2 already has a disk). 
    static private List<uint[]> motionList = new List<uint[]>(); 

    /*
    void Start()
    {
        uint numDisk = 4; // Number of Disk 
        uint startRod = 0; 
        uint endRod = 2; 
        uint auxRod = 1;

        diskName = new char[numDisk]; 
        for (uint i = 0; i < numDisk; i++)
        {
            diskName[i] = (char)('A' + i); 
        }
        stepList.Add(new uint[numDisk]); 

        TowerOfHanoi(numDisk, startRod, endRod, auxRod);
        PrintListUint(stepList);
        PrintListUint(motionList);
    } */

    public static (List<uint[]>, List<uint[]>) GetList(uint n, uint startRod, uint endRod, uint auxRod)
    {
        List<uint[]> stepList = new List<uint[]> { new uint[n] };
        List<uint[]> motionList = new List<uint[]>();
        TowerOfHanoi(n, startRod, endRod, auxRod, stepList, motionList); 
        return (stepList, motionList); 
    }

    static void TowerOfHanoi(uint n, uint startRod, uint endRod, uint auxRod, List<uint[]> stepList, List<uint[]> motionList)
    {
        if (n == 0)
        {
            return;
        }
        TowerOfHanoi(n - 1, startRod, auxRod, endRod, stepList, motionList);
        AssignStepListAndMotionList(n, endRod, stepList, motionList); 
        // Debug.Log("Move Disk" + diskName[n-1] + " from rod " + startRod + " to rod " + endRod); 
        TowerOfHanoi(n - 1, auxRod, endRod, startRod, stepList, motionList); 
    } 

    static void AssignStepListAndMotionList(uint n, uint endRod, List<uint[]> stepList, List<uint[]> motionList)
    {
        uint[] thisStpe = (uint[])stepList[stepList.Count - 1].Clone();
        thisStpe[n - 1] = endRod;
        stepList.Add(thisStpe);
        uint height = 0;
        foreach (uint position in thisStpe) { height += (uint)(thisStpe[n - 1] == position ? 1 : 0); }
        motionList.Add(new uint[] { n - 1, endRod, height - 1 });
    }

    void PrintListUint(List<uint[]> listUint)
    {
        uint step = 0;
        foreach (uint[] item in listUint)
        {
            string str = "Step" + step + ": "; 
            foreach (uint position in item)
            {
                str += (position + " "); 
            }
            Debug.Log(str);
            step++; 
        }
    }
}
