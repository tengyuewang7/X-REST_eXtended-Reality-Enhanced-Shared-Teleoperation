using UnityEditor;

namespace JarvenFramework.ResourceModule
{
    public class EditorResManager : Singleton<EditorResManager>
    {
        public T GetAssetRelative<T>(string resPath) where T : UnityEngine.Object
        {
            return AssetDatabase.LoadAssetAtPath<T>(resPath);
        }
    }
}