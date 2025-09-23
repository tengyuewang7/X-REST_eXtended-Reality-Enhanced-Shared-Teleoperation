using JarvenFramework;
using JarvenFramework.ResModule;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.InputSystem.Interactions;

public class PointCloudSystem : Singleton<PointCloudSystem>, IGameModule
{
    private int maxChunkSize = 65535;
    private GameObject _root;
    private List<GameObject> _elems;
    private GameObject _elemsTemplate;
    
    public GameObject Root { get { return _root; } }
    public void Initialize()
    {
        _elems = new List<GameObject>();
        _root = new GameObject("PointCloudRoot");
        _root.transform.position = new Vector3(0, 0, 0);
        _root.transform.rotation = Quaternion.identity;  
        _elemsTemplate = ResManager.Instance.LoadAsset<GameObject>("Assets/Prefabs/PointCloudElemTemplate.prefab");
        EventSystem.Instance.AppEvent.AddListener<List<float>>(AppEventId.ON_STREAM_UPDATE, Render);
    
    }

    public void Release()
    {
        EventSystem.Instance.AppEvent.RemoveListener<List<float>>(AppEventId.ON_STREAM_UPDATE, Render);
    }

    public void Render(List<float> lisVertices)
    {
        int nPoints = 0;
        int nChunks = 0;
        //_root.transform.position = SettingsSystem.Instance.RobotPosition;
        //_root.transform.rotation = SettingsSystem.Instance.RobotRotation;
        if (lisVertices != null)
        {
            nPoints = lisVertices.Count / 3;
            nChunks = 1 + nPoints / maxChunkSize;
        }

        if (_elems.Count < nChunks)
        {
            AddElems(nChunks - _elems.Count);
        }
        else
        {
            RemoveElems(_elems.Count - nChunks);
        }
        int offset = 0;
        for (int i = 0; i < nChunks; ++i)
        {
            int nPointsToRender = System.Math.Min(maxChunkSize, nPoints - offset);
            ElemRenderer(_elems[i], lisVertices, nPointsToRender, offset);
            offset += nPointsToRender;
        }
    }

    private void AddElems(int nElems)
    {
        for (int i = 0; i < nElems; ++i)
        {
            GameObject newElem = GameObject.Instantiate(_elemsTemplate);
            newElem.transform.parent = _root.transform;          
            newElem.transform.localPosition = Vector3.zero;
            newElem.transform.localRotation = Quaternion.identity;
            newElem.transform.localScale = Vector3.one;
            _elems.Add(newElem);
        }
    }

    private void RemoveElems(int nElems)
    {
        for (int i = 0; i < nElems; ++i)
        {
            GameObject.Destroy(_elems[0]);
            _elems.Remove(_elems[0]);
        }
    }

    private void ElemRenderer(GameObject obj, List<float> lisVertices, int nPointsToRender, int nPointsRendered)
    {
        if (lisVertices.Count > 0)
        {
            int nPoints = System.Math.Min(nPointsToRender, (lisVertices.Count / 3) - nPointsRendered);
            nPoints = System.Math.Min(nPoints, 65535);
            Mesh mesh = obj.GetComponent<Mesh>();
            if (mesh != null)
            {
                GameObject.Destroy(mesh);
            }
            Vector3[] points = new Vector3[nPoints];
            int[] indices = new int[nPoints];
            Color[] colors = new Color[nPoints];

            for (int i = 0; i < nPoints; ++i)
            {
                int ptIdx = 3 * (nPointsRendered + i);
                points[i] = new Vector3(lisVertices[ptIdx + 0], lisVertices[ptIdx + 2], lisVertices[ptIdx + 1]);
                indices[i] = i;
            }
            mesh = new Mesh();
            mesh.vertices = points;
            Debug.Log(points[0]);
            mesh.colors = colors;
            mesh.SetIndices(indices, MeshTopology.Points, 0);
            obj.GetComponent<MeshFilter>().mesh = mesh;
        }
    }
}