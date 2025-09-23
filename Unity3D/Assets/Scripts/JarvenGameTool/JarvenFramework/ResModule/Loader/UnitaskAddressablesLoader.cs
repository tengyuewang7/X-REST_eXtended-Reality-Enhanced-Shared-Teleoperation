using Cysharp.Threading.Tasks;
using System;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;
using System.Threading;

namespace JarvenFramework.ResModule
{
    public class UniTaskAddressablesLoader
    {

        public T LoadAsset<T>(string assetPath) where T : UnityEngine.Object
        {
            if (string.IsNullOrEmpty(assetPath))
            {
                return null;
            }

            AsyncOperationHandle<T> asyncOperationHandle = Addressables.LoadAssetAsync<T>(assetPath);
            T ret = asyncOperationHandle.WaitForCompletion();
            return ret;
        }

        public void LoadAssetAsync<T>(string assetPath, Action<T> callback) where T : UnityEngine.Object
        {
            if (string.IsNullOrEmpty(assetPath))
            {
                return;
            }
            UniTaskLoadAssetAsync(assetPath, callback).Forget();
        }


        /// <summary>
        /// 存在的问题是如果一个资产多次引用的话一次性全给卸了，加计数器？
        /// </summary>
        /// <param name="assetPath"></param>
        public void UnLoad(string assetPath)
        {
            Addressables.Release(assetPath);
        }


        private async UniTaskVoid UniTaskLoadAssetAsync<T>(string assetPath, Action<T> callback)
        {
            AsyncOperationHandle<T> handle = Addressables.LoadAssetAsync<T>(assetPath);
            T res = await handle;
            callback?.Invoke(res);
        }
        
        private async UniTaskVoid LoadAssetAsync<T>(string assetPath, Action<T> callback, CancellationToken token)
        {
            AsyncOperationHandle<T> handle = Addressables.LoadAssetAsync<T>(assetPath);
            T res = await handle.WithCancellation(token);
            callback?.Invoke(res);
        }

        private async UniTaskVoid LoadAssetAsync<T>(string assetPath, Action<T> callback, IProgress<float> progress)
        {
            AsyncOperationHandle<T> handle = Addressables.LoadAssetAsync<T>(assetPath);
            T res = await handle.ToUniTask(progress:progress);
            callback?.Invoke(res);
        }
    }
}
