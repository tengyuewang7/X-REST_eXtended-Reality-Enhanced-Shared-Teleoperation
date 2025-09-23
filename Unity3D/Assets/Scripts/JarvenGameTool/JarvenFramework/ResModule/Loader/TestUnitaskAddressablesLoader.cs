using Cysharp.Threading.Tasks;
using System;
using System.Collections.Generic;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;

namespace JarvenFramework.ResModule
{
    public class TestUniTaskAddressablesLoader:IResourceLoader
    {
        private class Handle
        {
            public AsyncOperationHandle operationHandle;
            public int refCount;

            public static Handle CreateHandle(AsyncOperationHandle operationHandle)
            {
                Handle handle = new Handle();
                handle.refCount = 1;
                handle.operationHandle = operationHandle;
                return handle;
            }
        }

        private readonly Dictionary<string, Handle> _dicLoadHandle = new Dictionary<string, Handle>();

        public T LoadAsset<T>(string assetPath) where T : UnityEngine.Object
        {
            if (string.IsNullOrEmpty(assetPath))
            {
                return null;
            }

            T ret = null;

            if (_dicLoadHandle.TryGetValue(assetPath, out Handle cacheHandle))
            {
                cacheHandle.refCount++;
                if (cacheHandle.operationHandle.IsDone)
                {
                    ret = cacheHandle.operationHandle.Result as T;
                }
                else
                {
                    ret = cacheHandle.operationHandle.WaitForCompletion() as T;
                }
            }
            else
            {
                AsyncOperationHandle<T> asyncOperationHandle = Addressables.LoadAssetAsync<T>(assetPath);
                ret = asyncOperationHandle.WaitForCompletion();
                _dicLoadHandle[assetPath] = Handle.CreateHandle(asyncOperationHandle);
            }
            return ret;
        }

        public void LoadAssetAsync<T>(string assetPath, Action<T> callback, IProgress<float> progress = null) where T : UnityEngine.Object
        {
            if (string.IsNullOrEmpty(assetPath))
            {
                return;
            }

            if (_dicLoadHandle.TryGetValue(assetPath, out Handle cacheHandle))
            {
                cacheHandle.refCount++;
                if (cacheHandle.operationHandle.IsDone)
                {
                    callback?.Invoke(cacheHandle.operationHandle.Result as T);
                }
                else
                {
                    if (progress != null)
                    {
                        UniTaskLoadAssetAsync(assetPath, callback, progress).Forget();
                    }
                    else
                    {
                        UniTaskLoadAssetAsync(assetPath, callback).Forget();
                    }
                }
            }
            else
            {
                if (progress != null)
                {
                    UniTaskLoadAssetAsync(assetPath, callback, progress).Forget();
                }
                else
                {
                    UniTaskLoadAssetAsync(assetPath, callback).Forget();
                }
            }
        }

        public void Unload(string assetPath)
        {
            if (!string.IsNullOrEmpty(assetPath) && _dicLoadHandle.TryGetValue(assetPath, out Handle handle))
            {
                if (--handle.refCount <= 0)
                {
                    if (handle.operationHandle.IsValid())
                    {
                        Addressables.Release(handle.operationHandle);
                    }
                    _dicLoadHandle.Remove(assetPath);
                }
            }
        }

        public void UnloadAll()
        {
            foreach (var kvp in _dicLoadHandle)
            {
                if (kvp.Value.operationHandle.IsValid())
                {
                    Addressables.Release(kvp.Value.operationHandle);
                }
            }
            _dicLoadHandle.Clear();
        }

        private async UniTaskVoid UniTaskLoadAssetAsync<T>(string assetPath, Action<T> callback, IProgress<float> progress = null)
        {
            AsyncOperationHandle<T> handle = Addressables.LoadAssetAsync<T>(assetPath);
            if (progress != null)
            {
                T res = await handle.ToUniTask(progress: progress);
                callback?.Invoke(res);
            }
            else
            {
                T res = await handle;
                callback?.Invoke(res);
            }
        }

        /*
        private async UniTaskVoid LoadAssetAsync<T>(string assetPath, Action<T> callback, CancellationToken token)
        {
            AsyncOperationHandle<T> handle = Addressables.LoadAssetAsync<T>(assetPath);
            T res = await handle.WithCancellation(token);
            callback?.Invoke(res);
        }
        */
    }
}
