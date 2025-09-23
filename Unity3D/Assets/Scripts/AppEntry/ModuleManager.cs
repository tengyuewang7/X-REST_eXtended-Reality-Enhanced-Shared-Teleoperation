using System;
using System.Collections.Generic;
using UnityEngine;
using JarvenFramework;

public class GameModuleManager : Singleton<GameModuleManager>
{
    private static readonly List<IGameModule> _listModules = new List<IGameModule>();


    public void Initialize()
    {
        //AddGameModule(EventSystem.Instance);
        //AddGameModule(SettingsSystem.Instance);
        //AddGameModule(InfoSystem.Instance);
        //AddGameModule(FtpSystem.Instance);
        //AddGameModule(GpuPointCloudSystem.Instance);
        //AddGameModule(CoordinateSystem.Instance);

        //AddGameModule(NetworkSystem.Instance);
        //AddGameModule(RobotSystem.Instance);
        OnInit();
    }

    public void Release()
    {
        for (int i = 0; i < _listModules.Count; i++)
        {
            try
            {
                _listModules[i].Release();
            }
            catch (Exception e)
            {
                Debug.LogException(e);
            }
        }
    }

    private void OnInit()
    {
        for (int i = 0; i < _listModules.Count; i++)
        {
            try
            {
                _listModules[i].Initialize();
            }
            catch (Exception e)
            {
                Debug.LogException(e);
            }
        }
    }

    private void AddGameModule(IGameModule gameModule)
    {
        _listModules.Add(gameModule);
    }
}