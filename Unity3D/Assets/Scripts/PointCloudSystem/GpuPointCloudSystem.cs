using JarvenFramework;
using JarvenFramework.ResModule;
using System;
using System.Collections.Generic;
using UniRx;
using UnityEngine;

public class GpuPointCloudSystem : Singleton<GpuPointCloudSystem>, IGameModule
{
    public const int PointCloudPotNum = 65535;

    private GameObject _root;
    private GameObject _originPoint;

    private ComputeBuffer _pointPosBuffer;
    private ComputeBuffer _pointColBuffer;
    private IDisposable _pointCloudUpdateTask;
    
    public void Initialize()
    {
        _root = new GameObject("PointCloudRoot");
        GameObject _elemsTemplate = ResManager.Instance.LoadAsset<GameObject>("Assets/Prefabs/ComputerShaderPointCloudTemplate.prefab");
        GameObject newElem = GameObject.Instantiate(_elemsTemplate);

        newElem.transform.parent = _root.transform;
        _originPoint = GameObject.Instantiate(ResManager.Instance.LoadAsset<GameObject>("Assets/Prefabs/OriginSphere.prefab"));
        _originPoint.transform.parent = _root.transform;

        Material material = _elemsTemplate.GetComponent<MeshRenderer>().sharedMaterial;
        _pointPosBuffer = new ComputeBuffer(PointCloudPotNum, 12);
        
        material.SetBuffer("PointPos", _pointPosBuffer);
        _pointColBuffer = new ComputeBuffer(PointCloudPotNum, 16);
        material.SetBuffer("PointCol", _pointColBuffer);

        EventSystem.Instance.AppEvent.AddListener<bool>(AppEventId.ON_POINTCLOUD_UPDATE, TaskActive);
    }


    private void TaskActive(bool active)
    {
        _pointCloudUpdateTask?.Dispose();
        if (active)
        {
            _pointCloudUpdateTask = Observable.Interval(TimeSpan.FromMilliseconds(50)).Sample(TimeSpan.FromSeconds(1.5f)).Subscribe(_ =>
            {
                FtpSystem.Instance.Download(SettingsSystem.Instance.Asset.LocalFileName, SettingsSystem.Instance.Asset.FtpServerFileName);
                List<float> info = InfoSystem.Instance.Deserialize<List<float>>(SettingsSystem.Instance.Asset.LocalFileName);
                Render(info);
            });
        }
    }


    public void Release()
    {
        EventSystem.Instance.AppEvent.RemoveListener<bool>(AppEventId.ON_POINTCLOUD_UPDATE, TaskActive);
    }

    public void Render(List<float> info)
    {
        Matrix4x4 rot = new Matrix4x4();
        rot.SetTRS(SettingsSystem.Instance.RobotPosition, SettingsSystem.Instance.RobotRotation, new Vector3(1, 1, 1));

        if (info.Count > 0)
        {
            Vector3[] points = new Vector3[PointCloudPotNum];
            for (int i = 0; i < info.Count / 3; ++i)
            {
                int ptIdx = 3 * i;
                Vector3 point = new Vector3(info[ptIdx + 0], info[ptIdx + 2], info[ptIdx + 1]);
                // points[i] = new Vector3(info[ptIdx + 0] - info[0], info[ptIdx + 2] - info[2], info[ptIdx + 1] - info[1]);
                Vector4 point1 = new Vector4(point[0], point[1], point[2], 1);
                point1 = rot * point1;
                points[i] = new Vector3(point1[0], point1[1], point1[2]);              
            }
            // _originPoint.transform.position = points[0];
            Color[] colors = GetColors(PointCloudPotNum);
            _pointPosBuffer.SetData(points);
            _pointColBuffer.SetData(colors);
        }
    }


    private Color[] GetColors(int num)
    {
        Color[] colors = new Color[num];
        colors[0] = Color.green;
        for (int i = 1; i < colors.Length; i++)
        {
            colors[i] = Color.red;
        }

        return colors;
    }
}
