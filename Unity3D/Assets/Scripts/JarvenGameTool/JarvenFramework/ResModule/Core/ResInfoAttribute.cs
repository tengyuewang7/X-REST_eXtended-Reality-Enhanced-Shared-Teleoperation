using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JarvenFramework.ResModule
{
    /// <summary>
    /// 资源特性
    /// </summary>
    [AttributeUsage(AttributeTargets.Class)]
    public class ResInfoAttribute:Attribute
    {
        /// <summary>
        /// 资源路径
        /// </summary>
        public string assetPath;

        /// <summary>
        /// 是否异步
        /// </summary>
        public bool async = false;

        public ResInfoAttribute()
        {

        }

        public ResInfoAttribute(string assetPath)
        {
            this.assetPath = assetPath;
        }

        public ResInfoAttribute(string assetPath, bool async)
        {
            this.assetPath = assetPath;
            this.async = async;
        }
    }
}
