using JarvenFramework;
using System;
using System.Collections.Generic;


/// <summary>
/// 事件管理
/// </summary>
public class EventManager<E>
{
    public delegate void Callback();
    public delegate void Callback<T>(T arg1);
    public delegate void Callback<T, U>(T arg1, U arg2);
    public delegate void Callback<T, U, V>(T arg1, U arg2, V arg3);
    public delegate void Callback<T1, T2, T3, T4>(T1 arg1, T2 arg2, T3 arg3, T4 arg4);

    private class EventReceiver
    {
        public Delegate listener;
    }

    private Dictionary<E, List<EventReceiver>> _dicEvent = new Dictionary<E, List<EventReceiver>>();

    public void AddListener(E eventId, Callback listener)
    {
        AddEvent(eventId, listener);
    }

    public void AddListener<T>(E eventId, Callback<T> listener)
    {
        AddEvent(eventId, listener);
    }

    public void AddListener<T, N>(E eventId, Callback<T, N> listener)
    {
        AddEvent(eventId, listener);
    }

    public void AddListener<T, U, V>(E eventId, Callback<T, U, V> listener)
    {
        AddEvent(eventId, listener);
    }

    public void AddListener<T1, T2, T3, T4>(E eventId, Callback<T1, T2, T3, T4> listener)
    {
        AddEvent(eventId, listener);
    }

    public void Dispatch(E eventId)
    {
        ForEach(eventId, listener => { if (listener is Callback callback) callback.Invoke(); });
    }

    public void Dispatch<T>(E eventId, T param1)
    {
        ForEach(eventId, listener => { if (listener is Callback<T> callback) callback.Invoke(param1); });
    }

    public void Dispatch<T, N>(E eventId, T param1, N param2)
    {
        ForEach(eventId, listener => { if (listener is Callback<T, N> callback) callback.Invoke(param1, param2); });
    }

    public void Dispatch<T, U, V>(E eventId, T param1, U param2, V param3)
    {
        ForEach(eventId, listener => { if (listener is Callback<T, U, V> callback) callback.Invoke(param1, param2, param3); });
    }

    public void Dispatch<T1, T2, T3, T4>(E eventId, T1 param1, T2 param2, T3 param3, T4 param4)
    {
        ForEach(eventId, listener => { if (listener is Callback<T1, T2, T3, T4> callback) callback.Invoke(param1, param2, param3, param4); });
    }

    public void RemoveListener(E eventId, Callback listner)
    {
        RemoveEvent(eventId, listner);
    }

    public void RemoveListener<T>(E eventId, Callback<T> listner)
    {
        RemoveEvent(eventId, listner);
    }

    public void RemoveListener<T, N>(E eventId, Callback<T, N> listner)
    {
        RemoveEvent(eventId, listner);
    }

    public void RemoveListener<T, U, V>(E eventId, Callback<T, U, V> listner)
    {
        RemoveEvent(eventId, listner);
    }

    public void RemoveListener<T1, T2, T3, T4>(E eventId, Callback<T1, T2, T3, T4> listner)
    {
        RemoveEvent(eventId, listner);
    }

    public void RemoveListener(E eventId)
    {
        _dicEvent.Remove(eventId);
    }

    public void RemoveAll()
    {
        _dicEvent.Clear();
    }

    private void AddEvent(E eventId, Delegate listener)
    {
        EventReceiver receiver = new EventReceiver();
        receiver.listener = listener;

        List<EventReceiver> list = _dicEvent.GetValueAnyway(eventId);

        if (list != null)
        {
            list.Add(receiver);
        }
        else
        {
            _dicEvent[eventId] = new List<EventReceiver>() { receiver };
        }
    }

    private void RemoveEvent(E eventId, Delegate listener)
    {
        List<EventReceiver> list = _dicEvent.GetValueAnyway(eventId);
        if (list != null)
        {
            int listCount = list.Count;
            for (int i = 0; i < listCount; i++)
            {
                if (listener == list[i]?.listener)
                {
                    list[i] = null;
                    break;
                }
            }
        }
    }

    private void ForEach(E eventId, Action<Delegate> action)
    {
        List<EventReceiver> list = _dicEvent.GetValueAnyway(eventId);
        if (list != null)
        {
            int listCount = list.Count;
            for (int i = listCount - 1; i >= 0; i--)
            {
                EventReceiver receiver = list[i];
                if (receiver != null)
                {
                    action?.Invoke(receiver.listener);
                }
                else
                {
                    list.RemoveAt(i);
                }
            }
        }
    }
}
