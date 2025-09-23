using System.Collections.Generic;
using UnityEngine;

namespace JarvenFramework
{
    public class GameObjectPool
    {
        private GameObject _template;
        private readonly Stack<GameObject> _pool = new Stack<GameObject>();

        public GameObjectPool(GameObject template)
        {
            this._template = template;
        }

        public GameObject Get(Transform parent)
        {
            GameObject result = null;

            if (_pool.Count > 0)
            {
                result = _pool.Pop();
                if (result != null)
                {
                    result.SetActive(true);
                    result.transform.SetParent(parent, false);
                }
                else
                {
                    result = GameObject.Instantiate(_template, parent);
                }
            }
            else
            {
                result = GameObject.Instantiate(_template, parent);
            }

            return result;
        }

        public void Recycle(GameObject obj)
        {
            obj?.SetActive(false);
            _pool.Push(obj);
        }
    }
}