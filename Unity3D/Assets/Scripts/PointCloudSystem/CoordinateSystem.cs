using JarvenFramework;
using JarvenFramework.ResModule;
using UnityEngine;


public class CoordinateSystem : Singleton<CoordinateSystem>, IGameModule
{
    private GameObject _coordinateObj;
    private bool isCoordinateStart = false;

    public void Initialize()
    {
        EventSystem.Instance.AppEvent.AddListener<Vector3>(AppEventId.ON_COORDINATE_START, OnCoordinateStart);
        EventSystem.Instance.AppEvent.AddListener(AppEventId.ON_COORDINATE_FINISH, OnCoordinateFinish);
    }

    public void Release()
    {
        EventSystem.Instance.AppEvent.RemoveListener(AppEventId.ON_COORDINATE_FINISH, OnCoordinateFinish);
        EventSystem.Instance.AppEvent.RemoveListener<Vector3>(AppEventId.ON_COORDINATE_START, OnCoordinateStart);
    }

    private void OnCoordinateStart(Vector3 position)
    {
        if (isCoordinateStart)
        {
            return;
        }
        ResManager.Instance.LoadAssetAsync<GameObject>("Assets/Prefabs/Controller.prefab", (obj) =>
        {
            _coordinateObj = GameObject.Instantiate(obj);
            _coordinateObj.transform.localPosition = position;  
            isCoordinateStart = true;   
            EventSystem.Instance.AppEvent.RemoveListener<Vector3>(AppEventId.ON_COORDINATE_START, OnCoordinateStart);
        });
    }

    private void OnCoordinateFinish()
    {
        Debug.Log(_coordinateObj.transform.position);
        SettingsSystem.Instance.RobotPosition = _coordinateObj.transform.position;
        SettingsSystem.Instance.RobotRotation = _coordinateObj.transform.rotation;
    }
}
