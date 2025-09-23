using System;

namespace JarvenFramework.ResModule
{
    public interface IResourceLoader
    {
        T LoadAsset<T>(string assetPath) where T : UnityEngine.Object;
        void LoadAssetAsync<T>(string assetPath, Action<T> callback, IProgress<float> progress) where T : UnityEngine.Object;
        void Unload(string assetPath);
        void UnloadAll();
    }
}
