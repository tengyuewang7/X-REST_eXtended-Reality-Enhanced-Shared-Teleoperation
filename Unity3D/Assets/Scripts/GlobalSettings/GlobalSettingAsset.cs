using UnityEngine;

[CreateAssetMenu(fileName = "Data", menuName = "ScriptableObjects/GlobalSettingAsset", order = 1)]
public class GlobalSettingAsset: ScriptableObject
{
    [SerializeField]
    public string FtpIp;

    [SerializeField]
    public string FtpUser;

    [SerializeField]
    public string FtpPassword;

    [SerializeField]
    public string FtpCatalog;

    [SerializeField]
    public string FtpServerFileName;

    [SerializeField]
    public string LocalFileName;

    [SerializeField]
    public Quaternion RobotRotation;

    [SerializeField]
    public Vector3 RobotPosition;
}