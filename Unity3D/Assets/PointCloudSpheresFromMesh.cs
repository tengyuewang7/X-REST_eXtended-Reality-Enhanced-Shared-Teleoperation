using System;
using System.Collections.Generic;
using UnityEngine;

public class PointCloudSpheresFromMesh : MonoBehaviour
{
    [Header("Mesh Input")]
    public Mesh sourceMesh;                  // 直接拖入 Mesh
    public MeshFilter sourceMeshFilter;      // 或拖入带 MeshFilter 的物体
    public bool transformToWorld = false;    // 使用 MeshFilter 的世界矩阵

    //[Header("Sampling")]
    public enum SamplingMode { IIDUniform, BlueNoise }
    public SamplingMode samplingMode = SamplingMode.IIDUniform;

    [Min(1)] public int pointCount = 5000;  // 目标点数（蓝噪声模式下是上限）
    public int randomSeed = 12345;
    [Tooltip("蓝噪声：点与点之间的最小距离（米）")]
    public float minSpacing = 0.02f;
    [Tooltip("蓝噪声：为每个点尝试的最大次数")]
    public int maxTrialsPerPoint = 30;
    [Tooltip("为视觉效果添加小抖动")]
    public float jitterStd = 0.0f;

    [Header("Sphere Visualization")]
    public float sphereRadius = 0.01f;
    public Color sphereColor = Color.white;
    public Material sphereMaterial = null; 

    System.Random rng; 

    public bool goUpdate = false; 

    void Start()
    {
        // 每 5 秒重采样一次（可改）
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
        if (!mesh) { Debug.LogError("请在 Inspector 拖入 sourceMesh 或 sourceMeshFilter。"); return; }

        rng = new System.Random(randomSeed);
        Matrix4x4 trs = (transformToWorld && sourceMeshFilter)
            ? sourceMeshFilter.transform.localToWorldMatrix
            : Matrix4x4.identity;

        // 顶点/三角形（需要 Read/Write；若未开，请到导入设置勾选）
        var verts = mesh.vertices;
        var tris = mesh.triangles;

        Vector3[] points =
            (samplingMode == SamplingMode.IIDUniform)
            ? Sample_IIDUniform(verts, tris, pointCount, trs, jitterStd)
            : Sample_BlueNoise(verts, tris, pointCount, trs, minSpacing, maxTrialsPerPoint, jitterStd);

        // 清理旧容器
        const string containerName = "PointCloudSpheres";
        var old = transform.Find(containerName);
#if UNITY_EDITOR
        if (old) { if (!Application.isPlaying) DestroyImmediate(old.gameObject); else Destroy(old.gameObject); }
#else
        if (old) Destroy(old.gameObject);
#endif

        // 可视化
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

        Debug.Log($"[{samplingMode}] 生成 {points.Length} 个点云小球。");
    }

    // ============ 表面积均匀（IID） ============
    Vector3[] Sample_IIDUniform(Vector3[] verts, int[] tris, int n, Matrix4x4 trs, float jitter)
    {
        int triCount = tris.Length / 3;
        if (triCount == 0) throw new Exception("网格没有三角面。");

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

            // 重心坐标均匀
            float u = (float)rng.NextDouble(), v = (float)rng.NextDouble();
            if (u + v > 1f) { u = 1f - u; v = 1f - v; }
            float w = 1f - u - v;

            Vector3 p = u * p0 + v * p1 + w * p2;
            if (jitter > 0) p += RandNormal3(jitter);
            pts[k] = p;
        }
        return pts;
    }

    // ============ 蓝噪声（近似 Poisson-disk） ============
    Vector3[] Sample_BlueNoise(
        Vector3[] verts, int[] tris, int n, Matrix4x4 trs,
        float minDist, int maxTrialsPerPt, float jitter)
    {
        // 先生成 3n 候选（IIDUniform），再做拒绝筛选（dart throwing）
        int candidatesN = Mathf.Max(n * 3, n + 32);
        var candidates = Sample_IIDUniform(verts, tris, candidatesN, trs, 0f);

        // 用体素网格做近邻加速
        float cell = minDist / Mathf.Sqrt(3f); // 立方体网格，保证同/邻格覆盖
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
                trials = 0; // 成功后重置
            }
            else
            {
                trials++;
                if (trials > maxTrialsPerPt && chosen.Count > 0)
                {
                    // 若长时间塞不进新点，适当放宽：从 IID 再拿一些
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
        // Box–Muller
        double u1 = 1.0 - UnityEngine.Random.value;
        double u2 = 1.0 - UnityEngine.Random.value;
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }
}
