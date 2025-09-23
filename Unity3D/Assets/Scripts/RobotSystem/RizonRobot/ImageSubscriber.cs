using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class ImageSubscriber : MonoBehaviour
{
    private string _topicName = "camera/color/image_raw";
    private RawImage rawImage;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(_topicName, ReceiveCallback);
        rawImage = GetComponent<RawImage>();
    }

    private void ReceiveCallback(ImageMsg msg)
    {
        rawImage.texture = msg.ToTexture2D();
    }

    void Update()
    {

    }
}
