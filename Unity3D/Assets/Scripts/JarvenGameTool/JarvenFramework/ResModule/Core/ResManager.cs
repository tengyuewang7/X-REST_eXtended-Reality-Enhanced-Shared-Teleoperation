using System;

namespace JarvenFramework.ResModule
{
    public class ResManager : Singleton<ResManager>, IResourceLoader
    {
        private IResourceLoader _loader;

        public void SetupLoader(IResourceLoader loader)
        {
            this._loader = loader;
        }

        public T LoadAsset<T>(string assetPath) where T : UnityEngine.Object
        {
            SetupLoader();
            return _loader?.LoadAsset<T>(assetPath);
        }

        public void LoadAssetAsync<T>(string assetPath, Action<T> callback, IProgress<float> onProgress = null) where T : UnityEngine.Object
        {
            SetupLoader();
            _loader?.LoadAssetAsync<T>(assetPath, callback, onProgress);
        }

        public void Unload(string assetPath)
        {
            SetupLoader();
            _loader?.Unload(assetPath);
        }

        public void UnloadAll()
        {
            SetupLoader();
            _loader?.UnloadAll();
        }

        private void SetupLoader()
        {
            if (this._loader == null)
            {
                _loader = new TestUniTaskAddressablesLoader();
            }
        }
    }
}
