using UnityEngine;

namespace JarvenFramework.UIModule
{
    /// <summary>
    /// CanvasGroup扩展
    /// </summary>
    public static class CanvasGroupExtends
    {
        public static void SetActive(this CanvasGroup canvasGroup, bool active) 
        {
            if (canvasGroup != null)
            {
                canvasGroup.alpha = active? 1 : 0;
                canvasGroup.blocksRaycasts = active;
            }
        }
        public static void SetAlpha(this CanvasGroup canvasGroup, bool active)
        {
            if (canvasGroup != null)
            {
                canvasGroup.alpha = active ? 1 : 0;
            }
        }

        public static bool IsActive(this CanvasGroup canvasGroup)
        {
            if (canvasGroup == null)
            {
                return false;
            }

            return canvasGroup.alpha > 0;
        }
    }
}
