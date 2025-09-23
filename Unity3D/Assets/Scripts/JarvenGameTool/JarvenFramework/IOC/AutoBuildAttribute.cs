using System;


namespace JarvenFramework
{
    /// <summary>
    /// 自动实例化接口
    /// </summary>
    [AttributeUsage(AttributeTargets.Field)]
    public class AutoBuildAttribute : Attribute
    {
        public string name;

        public AutoBuildAttribute()
        {

        }
        public AutoBuildAttribute(string name)
        {
            this.name = name;
        }
    }
}
