using System.Collections.Generic;
using UnityEngine.Events;
using UnityEngine.EventSystems;

namespace JarvenFramework
{
    /// <summary>
    /// �¼���������չ
    /// </summary>
    public static class EventTriggerExtends
    {
        /// <summary>
        /// ��Ӽ���
        /// </summary>
        public static EventTrigger AddListener(this EventTrigger trigger, EventTriggerType type, UnityAction<BaseEventData> action)
        {
            if (trigger.triggers == null)
            {
                trigger.triggers = new List<EventTrigger.Entry>();
            }

            EventTrigger.Entry target = null;

            for (int i = 0; i < trigger.triggers.Count; i++)
            {
                EventTrigger.Entry entry = trigger.triggers[i];

                if (entry != null && entry.eventID == type)
                {
                    target = entry;

                    break;
                }
            }

            if (target == null)
            {
                target = new EventTrigger.Entry()
                {
                    eventID = type
                };

                trigger.triggers.Add(target);
            }

            if (target.callback == null)
            {
                target.callback = new EventTrigger.TriggerEvent();
            }

            target.callback.AddListener(action);

            return trigger;
        }

        /// <summary>
        /// �Ƴ�����
        /// </summary>
        public static EventTrigger RemoveListener(this EventTrigger trigger, EventTriggerType type, UnityAction<BaseEventData> action)
        {
            if (trigger.triggers != null)
            {
                for (int i = 0; i < trigger.triggers.Count; i++)
                {
                    EventTrigger.Entry entry = trigger.triggers[i];

                    if (entry != null && entry.eventID == type && entry.callback != null)
                    {
                        entry.callback.RemoveListener(action);
                    }
                }
            }

            return trigger;
        }
    }
}