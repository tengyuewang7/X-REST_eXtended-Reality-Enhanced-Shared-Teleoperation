using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class RealsenseImageSubscriber : MonoBehaviour
{
    public Material m;
    private Texture2D mergedTexture;
    private string _topicName = "/camera_images";    // D435: 640*480

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(_topicName, ReceiveCallback);
    }

    private void ReceiveCallback(ImageMsg msg)
    {

        m.mainTexture = msg.ToTexture2D();
    }

    UnityEngine.Color[] ResizeImage(Texture2D source, float scale=0.5f)
    {
        int originalWidth = source.width;
        int originalHeight = source.height;
        int newWidth = Mathf.FloorToInt(scale * originalWidth);
        int newHeight = Mathf.FloorToInt(scale * originalHeight);
        UnityEngine.Color[] originalPixels = source.GetPixels();
        UnityEngine.Color[] resizedPixels = new UnityEngine.Color[newWidth * newHeight];
        for (int y = 0; y < newHeight; y++)
        {
            for (int x = 0; x < newWidth; x++)
            {
                // 计算采样的原始像素坐标
                int originalX1 = x * 2;
                int originalY1 = y * 2;
                int originalX2 = originalX1 + 1;
                int originalY2 = originalY1 + 1;

                // 获取四个相邻像素
                UnityEngine.Color p1 = originalPixels[originalY1 * originalWidth + originalX1];
                UnityEngine.Color p2 = originalPixels[originalY1 * originalWidth + originalX2];
                UnityEngine.Color p3 = originalPixels[originalY2 * originalWidth + originalX1];
                UnityEngine.Color p4 = originalPixels[originalY2 * originalWidth + originalX2];

                // 平均四个像素的颜色值
                UnityEngine.Color averageColor = (p1 + p2 + p3 + p4) / 4.0f;

                // 设置缩小后的像素颜色
                resizedPixels[y * newWidth + x] = averageColor;
            }
        }
        return resizedPixels;
    }

    void Update()
    {
    }
}
