using System;
using System.Collections.Generic;
using UnityEngine;

public class PointCloudSpheresFromMesh : MonoBehaviour
{
    [Header("Mesh Input")]
    public Mesh sourceMesh;                  // ֱ������ Mesh
    public MeshFilter sourceMeshFilter;      // ������� MeshFilter ������
    public bool transformToWorld = false;    // ʹ�� MeshFilter ���������

    //[Header("Sampling")]
    public enum SamplingMode { IIDUniform, BlueNoise }
    public SamplingMode samplingMode = SamplingMode.IIDUniform;

    [Min(1)] public int pointCount = 5000;  // Ŀ�������������ģʽ�������ޣ�
    public int randomSeed = 12345;
    [Tooltip("�������������֮�����С���루�ף�")]
    public float minSpacing = 0.02f;
    [Tooltip("��������Ϊÿ���㳢�Ե�������")]
    public int maxTrialsPerPoint = 30;
    [Tooltip("Ϊ�Ӿ�Ч�����С����")]
    public float jitterStd = 0.0f;

    [Header("Sphere Visualization")]
    public float sphereRadius = 0.01f;
    public Color sphereColor = Color.white;
    public Material sphereMaterial = null; 

    System.Random rng; 

    public bool goUpdate = false; 

    void Start()
    {
        // ÿ 5 ���ز���һ�Σ��ɸģ�
        //GenerateSpheres();
        //InvokeRepeating(nameof(GenerateSpheres), 0f, 5f);
    }

    private void Update()
    {
        if (goUpdate)
        {
            GenerateSpheres();
            goUpdate = false;
        }
    }

    [ContextMenu("Generate Spheres")]
    public void GenerateSpheres()
    {
        Mesh mesh = sourceMesh;
        if (mesh == null && sourceMeshFilter) mesh = sourceMeshFilter.sharedMesh;
        if (!mesh) { Debug.LogError("���� Inspector ���� sourceMesh �� sourceMeshFilter��"); return; }

        rng = new System.Random(randomSeed);
        Matrix4x4 trs = (transformToWorld && sourceMeshFilter)
            ? sourceMeshFilter.transform.localToWorldMatrix
            : Matrix4x4.identity;

        // ����/�����Σ���Ҫ Read/Write����δ�����뵽�������ù�ѡ��
        var verts = mesh.vertices;
        var tris = mesh.triangles;

        Vector3[] points =
            (samplingMode == SamplingMode.IIDUniform)
            ? Sample_IIDUniform(verts, tris, pointCount, trs, jitterStd)
            : Sample_BlueNoise(verts, tris, pointCount, trs, minSpacing, maxTrialsPerPoint, jitterStd);

        // ���������
        const string containerName = "PointCloudSpheres";
        var old = transform.Find(containerName);
#if UNITY_EDITOR
        if (old) { if (!Application.isPlaying) DestroyImmediate(old.gameObject); else Destroy(old.gameObject); }
#else
        if (old) Destroy(old.gameObject);
#endif

        // ���ӻ�
        var container = new GameObject(containerName).transform;
        container.SetParent(transform, false);
        foreach (var p in points)
        {
            var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.transform.SetParent(container, false);
            s.transform.position = p;
            s.transform.localScale = Vector3.one * (2f * sphereRadius); 
            s.transform.localScale = new Vector3(s.transform.localScale.x / transform.localScale.x,
                s.transform.localScale.y / transform.localScale.y, s.transform.localScale.z / transform.localScale.z);
            var mr = s.GetComponent<MeshRenderer>();
            if (mr)
            {
                mr.material = new Material(Shader.Find("Standard"));
                mr.material.color = sphereColor;
            }
            var col = s.GetComponent<Collider>();
#if UNITY_EDITOR
            if (!Application.isPlaying) DestroyImmediate(col); else if (col) Destroy(col);
#else
            if (col) Destroy(col);
#endif
        }

        Debug.Log($"[{samplingMode}] ���� {points.Length} ������С��");
    }

    // ============ ��������ȣ�IID�� ============
    Vector3[] Sample_IIDUniform(Vector3[] verts, int[] tris, int n, Matrix4x4 trs, float jitter)
    {
        int triCount = tris.Length / 3;
        if (triCount == 0) throw new Exception("����û�������档");

        var areas = new float[triCount];
        double sum = 0;
        for (int i = 0; i < triCount; i++)
        {
            int a = tris[i * 3], b = tris[i * 3 + 1], c = tris[i * 3 + 2];
            Vector3 p0 = trs.MultiplyPoint3x4(verts[a]);
            Vector3 p1 = trs.MultiplyPoint3x4(verts[b]);
            Vector3 p2 = trs.MultiplyPoint3x4(verts[c]);
            float area = 0.5f * Vector3.Cross(p1 - p0, p2 - p0).magnitude;
            areas[i] = area; sum += area;
        }
        var cdf = new double[triCount]; double acc = 0;
        for (int i = 0; i < triCount; i++) { acc += areas[i]; cdf[i] = acc / sum; }

        var pts = new Vector3[n];
        for (int k = 0; k < n; k++)
        {
            double r = rng.NextDouble();
            int t = Array.BinarySearch(cdf, r); if (t < 0) t = ~t; if (t >= triCount) t = triCount - 1;

            int a = tris[t * 3], b = tris[t * 3 + 1], c = tris[t * 3 + 2];
            Vector3 p0 = trs.MultiplyPoint3x4(verts[a]);
            Vector3 p1 = trs.MultiplyPoint3x4(verts[b]);
            Vector3 p2 = trs.MultiplyPoint3x4(verts[c]);

            // �����������
            float u = (float)rng.NextDouble(), v = (float)rng.NextDouble();
            if (u + v > 1f) { u = 1f - u; v = 1f - v; }
            float w = 1f - u - v;

            Vector3 p = u * p0 + v * p1 + w * p2;
            if (jitter > 0) p += RandNormal3(jitter);
            pts[k] = p;
        }
        return pts;
    }

    // ============ ������������ Poisson-disk�� ============
    Vector3[] Sample_BlueNoise(
        Vector3[] verts, int[] tris, int n, Matrix4x4 trs,
        float minDist, int maxTrialsPerPt, float jitter)
    {
        // ������ 3n ��ѡ��IIDUniform���������ܾ�ɸѡ��dart throwing��
        int candidatesN = Mathf.Max(n * 3, n + 32);
        var candidates = Sample_IIDUniform(verts, tris, candidatesN, trs, 0f);

        // ���������������ڼ���
        float cell = minDist / Mathf.Sqrt(3f); // ���������񣬱�֤ͬ/�ڸ񸲸�
        var grid = new Dictionary<(int, int, int), List<int>>(candidatesN);
        var chosen = new List<Vector3>(n);

        int trials = 0;
        for (int i = 0; i < candidates.Length && chosen.Count < n; i++)
        {
            var p = candidates[i];
            if (IsFarEnough(p, chosen, grid, cell, minDist))
            {
                chosen.Add(jitter > 0 ? p + RandNormal3(jitter) : p);
                PlaceIntoGrid(p, grid, cell, chosen.Count - 1);
                trials = 0; // �ɹ�������
            }
            else
            {
                trials++;
                if (trials > maxTrialsPerPt && chosen.Count > 0)
                {
                    // ����ʱ���������µ㣬�ʵ��ſ��� IID ����һЩ
                    trials = 0;
                }
            }
        }
        return chosen.ToArray();
    }

    bool IsFarEnough(Vector3 p, List<Vector3> chosen,
                     Dictionary<(int, int, int), List<int>> grid, float cell, float minDist)
    {
        var key = CellOf(p, cell);
        for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    var nk = (key.Item1 + dx, key.Item2 + dy, key.Item3 + dz);
                    if (!grid.TryGetValue(nk, out var list)) continue;
                    for (int li = 0; li < list.Count; li++)
                    {
                        if ((chosen[list[li]] - p).sqrMagnitude < minDist * minDist)
                            return false;
                    }
                }
        return true;
    }

    void PlaceIntoGrid(Vector3 p, Dictionary<(int, int, int), List<int>> grid, float cell, int idx)
    {
        var key = CellOf(p, cell);
        if (!grid.TryGetValue(key, out var list))
        {
            list = new List<int>(4);
            grid[key] = list;
        }
        list.Add(idx);
    }

    (int, int, int) CellOf(Vector3 p, float cell)
    {
        return (Mathf.FloorToInt(p.x / cell),
                Mathf.FloorToInt(p.y / cell),
                Mathf.FloorToInt(p.z / cell));
    }

    Vector3 RandNormal3(float std)
    {
        return new Vector3(BoxMuller() * std, BoxMuller() * std, BoxMuller() * std);
    }
    float BoxMuller()
    {
        // Box�CMuller
        double u1 = 1.0 - UnityEngine.Random.value;
        double u2 = 1.0 - UnityEngine.Random.value;
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
