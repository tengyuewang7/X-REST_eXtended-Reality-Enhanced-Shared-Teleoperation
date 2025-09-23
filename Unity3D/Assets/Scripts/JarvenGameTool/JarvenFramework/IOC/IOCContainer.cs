using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace JarvenFramework
{

    public static class IOCContainer
    {
        private static readonly Dictionary<Type, Dictionary<string, ITypeNode>> _dic = new Dictionary<Type, Dictionary<string, ITypeNode>>(16);


        public static void Regist<T, V>(string name = null) where V : class, T, new()
        {
            Regist<T>(name, new NormalTypeNode(typeof(V)));
        }

        public static void Regist<T, V>(object obj) where V : class, T, new()
        {
            Regist<T, V>(obj?.ToString());
        }

        public static void RegistSingleton<T, V>(V obj, string name) where V : class, T, new()
        {
            SingletonTypeNode node = null;
            if (obj != null)
            {
                node = new SingletonTypeNode(obj);
            }
            else
            {
                node = new SingletonTypeNode(typeof(V));
            }
            Regist<T>(name, node);
        }

        public static void RegistSingleton<T, V>() where V : class, T, new()
        {
            SingletonTypeNode node = new SingletonTypeNode(typeof(V));

            Regist<T>(string.Empty, node);
        }

        public static void RegistSingleton<T, V>(string name) where V : class, T, new()
        {
            RegistSingleton<T, V>(null, name);
        }

        public static void RegistSingleton<T, V>(V obj) where V : class, T, new()
        {
            RegistSingleton<T, V>(obj, string.Empty);
        }

        public static T Resolve<T>(string name = null)
        {
            return (T)Resolve(typeof(T), name);
        }

        public static T Resolve<T>(object obj)
        {
            return Resolve<T>(obj?.ToString());
        }

        public static object Resolve(Type type, string name = null)
        {
            object res = null;

            name = string.IsNullOrEmpty(name) ? string.Empty : name;

            Dictionary<string, ITypeNode> dic = null;

            if (_dic.TryGetValue(type, out dic))
            {
                ITypeNode node = null;

                dic?.TryGetValue(name, out node);

                if (node is NormalTypeNode)
                {
                    NormalTypeNode normalTypeNode = node as NormalTypeNode;

                    res = Activator.CreateInstance(normalTypeNode.objType);

                    GenerateInterfaceField(res);
                }
                else if (node is SingletonTypeNode)
                {
                    SingletonTypeNode singletonNode = node as SingletonTypeNode;

                    res = singletonNode.Obj;
                }
            }
            if (res == null)
            {
                Debug.LogErrorFormat(" ## Cls:IOCContainer Func:Resolve Type:{0}{1} Info:Unregistered", type, !string.IsNullOrEmpty(name) ? $" Name:{name}" : string.Empty);
            }
            return res;
        }

        private static void Regist<T>(string name, ITypeNode node)
        {
            name = string.IsNullOrEmpty(name) ? string.Empty : name;

            Type type = typeof(T);

            if (!_dic.TryGetValue(type, out Dictionary<string, ITypeNode> dic))
            {
                dic = new Dictionary<string, ITypeNode>();
                _dic[type] = dic;
            }
            dic[name] = node;
        }

        private static void GenerateInterfaceField(object obj)
        {
            GenerateInterfaceField(obj, obj?.GetType());
        }

        private static void GenerateInterfaceField(object obj, Type type)
        {
            if (obj == null || type == null)
            {
                return;
            }

            FieldInfo[] fieldInfos = type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);

            if (fieldInfos != null && fieldInfos.Length > 0)
            {
                for (int i = 0; i < fieldInfos.Length; ++i)
                {
                    FieldInfo fieldInfo = fieldInfos[i];

                    if (fieldInfo != null && fieldInfo.FieldType.IsInterface)
                    {
                        object[] autoAttrs = fieldInfo.GetCustomAttributes(typeof(AutoBuildAttribute), true);

                        string name = null;

                        if (autoAttrs != null && autoAttrs.Length > 0)
                        {
                            foreach (AutoBuildAttribute attr in autoAttrs)
                            {
                                if (attr != null)
                                {
                                    name = attr.name;
                                    break;
                                }
                            }
                            fieldInfo.SetValue(obj, Resolve(fieldInfo.FieldType, name));
                        }
                    }
                }
                GenerateInterfaceField(obj, type.BaseType);
            }
        }

        private interface ITypeNode
        {

        }

        private class SingletonTypeNode : ITypeNode
        {
            private object _obj;

            private Type _type;

            public object Obj
            {
                get
                {
                    if (_obj == null)
                    {
                        _obj = Activator.CreateInstance(_type);
                        GenerateInterfaceField(_obj);
                    }
                    return _obj;
                }
            }

            public SingletonTypeNode(Type objType)
            {
                this._type = objType;
            }
            public SingletonTypeNode(object obj)
            {
                this._obj = obj;
            }
        }

        private class NormalTypeNode : ITypeNode
        {
            public Type objType;
            public NormalTypeNode(Type objType)
            {
                this.objType = objType;
            }
        }
    }
}
