using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Move : MonoBehaviour
{
    public float translationDistance; 
    public float rotationDegree;
    public List<GameObject> objectList;
    public GameObject referenceObject;
    private GameObject marker;
    public GameObject prefabMarker;

    public void CreateMarker()
    {
        if (marker == null)
        {
            marker = Instantiate(prefabMarker, referenceObject.transform.position,
                referenceObject.transform.rotation);
        }
        else
        {
            Destroy(marker);
            marker = null;
        }
    }

    private void TranslateMoving(in Vector3 movement) 
    {
        foreach (GameObject obj in objectList) 
        {
            obj.transform.Translate(movement, Space.World);
        }
    }

    private void RotateMoving(in Vector3 axis, in float angle) 
    {
        foreach (GameObject obj in objectList) 
        {
            obj.transform.RotateAround(referenceObject.transform.position, 
                axis, angle);
        }
    }

    public void XPositiveTranslation() 
    {
        TranslateMoving(new Vector3(translationDistance, 0f, 0f));
    }

    public void XNegativeTranslation() 
    {
        TranslateMoving(new Vector3(-translationDistance, 0f, 0f));
    }

    public void YPositiveTranslation() 
    {
        TranslateMoving(new Vector3(0f, translationDistance, 0f));
    }

    public void YNegativeTranslation() 
    {
        TranslateMoving(new Vector3(0f, -translationDistance, 0f));
    }

    public void ZPositiveTranslation() 
    {
        TranslateMoving(new Vector3(0f, 0f, translationDistance));
    }

    public void ZNegativeTranslation() 
    {
        TranslateMoving(new Vector3(0f, 0f, -translationDistance));
    }
    
    public void XPositiveRotation() 
    {
        RotateMoving(referenceObject.transform.right, rotationDegree);
    }
    
    public void XNegativeeRotation() 
    {
        RotateMoving(-referenceObject.transform.right, rotationDegree);
    }

    public void YPositiveRotation() 
    {
        RotateMoving(referenceObject.transform.up, rotationDegree);

    }

    public void YNegativeRotation() 
    {
        RotateMoving(-referenceObject.transform.up, rotationDegree);
    }

    public void ZPositiveRotation() 
    {
        RotateMoving(referenceObject.transform.forward, rotationDegree);
    }

    public void ZNegativeRotation() 
    {
        RotateMoving(-referenceObject.transform.forward, rotationDegree);
    }

}
