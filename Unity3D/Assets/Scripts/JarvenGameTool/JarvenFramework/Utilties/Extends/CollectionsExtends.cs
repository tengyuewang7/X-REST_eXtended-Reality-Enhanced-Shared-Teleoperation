using System;
using System.Collections.Generic;

namespace JarvenFramework
{    /// <summary>
     /// 集合扩展
     /// </summary>
    public static class CollectionsExtends
    {
        public static bool TryAdd<T, V>(this Dictionary<T, V> dic, T key, V value, bool isOverride = true)
        {
            bool isSuccessAdd = true;
            V v = default(V);
            if (dic.TryGetValue(key, out v))
            {
                if (v != null && isOverride)
                {
                    dic[key] = value;
                }
                else
                {
                    isSuccessAdd = false;
                }
            }
            else
            {
                dic.Add(key, value);
            }
            return isSuccessAdd;
        }
        /// <summary>
        /// 尝试获取数组中的值
        /// </summary>
        public static bool TryGetValue<T>(this IList<T> arr, int index, out T t)
        {
            bool res = false;

            if (arr != null && index >= 0 && index < arr.Count)
            {
                t = arr[index];

                res = true;
            }
            else
            {
                t = default(T);
            }

            return res;
        }

        public static bool TryRemove<T, V>(this Dictionary<T, V> dic, T key)
        {
            if (dic.ContainsKey(key))
            {
                dic.Remove(key);
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// 尝试设置数组中的值
        /// </summary>
        public static bool TrySetValue<T>(this IList<T> arr, int index, T t)
        {
            bool res = false;
            if (arr != null && index >= 0 && index < arr.Count)
            {
                arr[index] = t;

                res = true;
            }
            return res;
        }

        /// <summary>
        /// 是否合法的索引
        /// </summary>
        public static bool IsValid<T>(this IList<T> arr, int index)
        {
            return arr != null && index >= 0 && index < arr.Count;
        }

        /// <summary>
        /// 获取数组值
        /// 若不存在返回默认
        /// </summary>
        public static T GetValueAnyway<T>(this IList<T> arr, int index)
        {
            arr.TryGetValue(index, out T t);

            return t;
        }

        /// <summary>
        /// 获取值
        /// 若不存在返回默认
        /// </summary>
        public static TValue GetValueAnyway<TKey, TValue>(this IDictionary<TKey, TValue> dic, TKey key)
        {
            if (key == null)
            {
                return default(TValue);
            }

            dic.TryGetValue(key, out TValue val);

            return val;
        }

        /// <summary>
        /// 合并数组
        /// </summary>
        public static T[] Combine<T>(this T[] src, T[] target)
        {
            if (src == null)
            {
                return target;
            }

            if (target == null)
            {
                return src;
            }

            T[] result = new T[src.Length + target.Length];

            Array.Copy(src, result, src.Length);
            Array.Copy(target, 0, result, src.Length, target.Length);

            return result;
        }

    }
}
