using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeAlpha : MonoBehaviour
{
    private Transform[] _transforms;
    private List<MeshRenderer> _allRenderer;
    // Start is called before the first frame update
    void Start()
    {
        _allRenderer = new List<MeshRenderer>();
        _transforms = GetComponentsInChildren<Transform>();
        foreach (Transform t in _transforms)
        {
            if(t.name=="default")
            {
                _allRenderer.Add(t.gameObject.GetComponent<MeshRenderer>());
            }
        }
        StartCoroutine("FadeFade");
    }


    IEnumerator FadeFade()
    {
        for (float f = 1f; f >= 0; f -= 0.03f)
        {
            foreach (MeshRenderer t in _allRenderer)
            {
                Material material = t.material;
                Color color = material.color;
                color.a = f;
                material.color = color;
            }
            yield return new WaitForSeconds(0.1f);
        }
    }


    /*
    void Update()
    {
        foreach (MeshRenderer t in _allRenderer)
        {
            Material material = t.material;
            Color color = material.color;
            color.a -= 0.05f;
            material.color = color;
        }

        foreach (MeshRenderer t in _allRenderer)
        {
            Debug.Log(t.material.color.a);
        }
    } */
}
