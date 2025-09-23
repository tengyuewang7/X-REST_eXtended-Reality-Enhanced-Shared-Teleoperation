using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AppEntry : MonoBehaviour
{
    // Start is called before the first frame update
    private void awake()
    {
        DontDestroyOnLoad(this.gameObject);
    }

    private void Start()
    {
        GameModuleManager.Instance.Initialize();
    }

    private void OnDestroy()
    {
        GameModuleManager.Instance.Release();
    }
}