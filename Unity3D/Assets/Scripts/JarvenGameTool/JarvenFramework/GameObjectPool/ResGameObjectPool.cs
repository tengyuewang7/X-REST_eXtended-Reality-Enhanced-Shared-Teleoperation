using System;
using System.Collections.Generic;
using UnityEngine;
using JarvenFramework.ResModule;
using UniRx;

namespace JarvenFramework
{

    public class ResGameObjectPool
    {
        private class ToDoItem
        {
            public string name;
            public Action<GameObject> callback;
            public ToDoItem(string name, Action<GameObject> callback)
            {
                this.name = name;
                this.callback = callback;
            }
        }

        private readonly Queue<ToDoItem> _toDoQueue = new Queue<ToDoItem>();
        private Dictionary<string, Stack<GameObject>> _pool = new Dictionary<string, Stack<GameObject>>();
        private IDisposable _createTask;

        public void Get(string name, Action<GameObject> callback)
        {
            ToDoItem item = new ToDoItem(name, callback);
            _toDoQueue.Enqueue(item);
            if (_createTask == null)
            {
                int waitFrame = 60;
                _createTask = Observable.EveryUpdate()
                    .Subscribe(_ => CreateTask(ref waitFrame));
            }
        }

        public void Remove(string name, GameObject obj)
        {
            if (obj == null)
            {
                return;
            }

            obj.SetActive(false);

            if (_pool.TryGetValue(name, out Stack<GameObject> stk))
            {
                stk.Push(obj);
            }
            else
            {
                stk = new Stack<GameObject>(64);
                stk.Push(obj);
                _pool[name] = stk;
            }
        }

        public void Destroy(string name, GameObject obj)
        {
            obj.SetActive(false);

            GameObject.Destroy(obj);
        }

        private void CreateTask(ref int waitFrame)
        {
            if (_toDoQueue.Count > 0)
            {
                ToDoItem item = _toDoQueue.Dequeue();

                bool fromPool = false;

                if (_pool.TryGetValue(item.name, out Stack<GameObject> stack))
                {
                    if (stack.Count > 0)
                    {
                        GameObject obj = stack.Pop();
                        obj.SetActive(true);
                        item.callback?.Invoke(obj);

                        fromPool = true;
                    }
                }

                if (!fromPool)
                {
                    ResManager.Instance.LoadAssetAsync<GameObject>(item.name, (prefab) =>
                    {
                        item.callback?.Invoke(GameObject.Instantiate(prefab));
                    });
                }

                waitFrame = 60;
            }
            else
            {
                if (--waitFrame <= 0)
                {
                    _createTask?.Dispose();
                    _createTask = null;
                }
            }
        }

    }
}
