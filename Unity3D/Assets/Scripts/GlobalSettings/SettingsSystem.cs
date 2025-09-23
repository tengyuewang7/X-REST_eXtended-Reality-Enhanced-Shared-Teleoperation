using JarvenFramework;
using JarvenFramework.ResModule;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections.Generic;
using UniRx;
using UnityEngine;

public class SettingsSystem : Singleton<SettingsSystem>, IGameModule, IMixedRealitySpeechHandler, IMixedRealityHandJointHandler
{
    private GlobalSettingAsset asset;
    private bool isNeedToGetRightHandIndexPosition = false;
    private Vector3 rightHandIndexPosition = Vector3.zero;


    public GlobalSettingAsset Asset
    {
        get { return asset; } 
    }
    public Vector3 RobotPosition
    {
        set { asset.RobotPosition = value; }
        get { return asset.RobotPosition; }
    }

    public Quaternion RobotRotation
    {
        set { asset.RobotRotation = value; }
        get { return asset.RobotRotation; }
    }
    public void Initialize()
    {
        CoreServices.InputSystem?.RegisterHandler<IMixedRealitySpeechHandler>(this);
        CoreServices.InputSystem?.RegisterHandler<IMixedRealityHandJointHandler>(this);

        asset = ResManager.Instance.LoadAsset<GlobalSettingAsset>("Assets/Scripts/GlobalSetting.asset");
    }

    public void OnHandJointsUpdated(InputEventData<IDictionary<TrackedHandJoint, MixedRealityPose>> eventData)
    {
        if (isNeedToGetRightHandIndexPosition && eventData.Handedness.Equals(Handedness.Right))
        {
            if (eventData.InputData.TryGetValue(TrackedHandJoint.IndexTip, out MixedRealityPose pose) && pose != null)
            {
                rightHandIndexPosition = pose.Position;
                
                isNeedToGetRightHandIndexPosition = false;
                EventSystem.Instance.AppEvent.Dispatch<Vector3>(AppEventId.ON_COORDINATE_START, rightHandIndexPosition);
            }
        }
    }

    public void OnSpeechKeywordRecognized(SpeechEventData eventData)
    {
        Debug.Log(eventData.Command.Keyword.ToLower());
        switch (eventData.Command.Keyword.ToLower())
        {
            case "point cloud update start":
                EventSystem.Instance.AppEvent.Dispatch(AppEventId.ON_POINTCLOUD_UPDATE, true);
                break;
            case "point cloud update finish":
                EventSystem.Instance.AppEvent.Dispatch(AppEventId.ON_POINTCLOUD_UPDATE, false);
                break;
            case "coordinate start":
                isNeedToGetRightHandIndexPosition = true;
                break;
            case "coordinate finish":
                EventSystem.Instance.AppEvent.Dispatch(AppEventId.ON_COORDINATE_FINISH);
                break;
        }
    }

    public void Release()
    {
        CoreServices.InputSystem?.UnregisterHandler<IMixedRealityHandJointHandler>(this);
        ResManager.Instance.Unload("Assets/Scripts/GlobalSetting.asset");
        CoreServices.InputSystem?.UnregisterHandler<IMixedRealitySpeechHandler>(this);
    }
}
