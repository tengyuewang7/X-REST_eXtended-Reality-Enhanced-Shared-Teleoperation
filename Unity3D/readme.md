## readme

### 项目工程文件夹梳理

Assets目录下存储所有工程资源与代码，主动开发的东西全扔这里

AdressableAssetsData为资源管理系统自动生成文件，由于使用原生aa系统作为底层所以不要动
MixedRealityToolkit.Generated为MRTK生成配置文件的存储，一般只需要考虑修改部分配置时才需要改动，具体见输入系统部分。
MRTK文件夹不需要动，除非需要一些预制体资源去复制。
Plugins中存储插件dll，后续需要外部dll也可以扔进这个地方。
Prefabs中存储所有资源文件，包括模型、贴图、预制体、mesh等等。
Scenes中存储场景文件，由于采用的是流式加载资源法其实一个主场景就行，需要单元测试的时候再开副场景。
Scripts中存储代码文件。



### 项目模块

#### 全局入口
为了减轻加载负担因此项目场景初始仅有一个全局入口组件，后续所有代码和资源均为流式加载，无全局静态与全局实例对象。

采用组合设计模式，将其他所有模块的初始化与卸载均在ModuleManager中完成。
功能模块需要继承IGameModule接口并实现Initialize()与Release()方法，之后在ModuleManager的Initialize()方法中AddGameModule。

目前存在功能逻辑模块与底层支持模块未明确区分的问题，不过可以通过人工控制来解决。

#### 事件系统
为了降低代码耦合度写的事件管理系统，基于发布订阅模式，封装了委托避免后续开发工作中对c#的delegate和action不理解导致出错。
目前功能单一的情况下统一视为AppEvent，不要修改EventSystem.cs和EventManeger.cs中的东西。
如果需要添加事件，在EventId.cs中添加即可

```c#
public enum AppEventId : int
{
    ...
    ON_TEST_UPDATE, 
    ...
}


public class A : MonoBehaviour
{
    Update() {
        EventSystem.Instance.AppEvent.Dispatch<Vector3>(EventId.ON_TEST_UPDATE, this.transform.position);
    }
}

public class B : MonoBehaviour
{
    OnStart() {
        // 增加监听
        EventSystem.Instance.AppEvent.AddListener<Vector3>(EventId.ON_TEST_UPDATE, CallbackFunction);
    }

    OnDestory() {
        // 删除监听
        EventSystem.Instance.AppEvent.RemoveListener<Vector3>(EventId.ON_TEST_UPDATE, CallbackFunction);
    }

    void CallbackFunction(Vector3 data) 
    {
        Debug.Log(data);
    }
}

```

##### 拓展：处理不同类型事件

##### 拓展：与MRTK的输入识别接口封装
目前暂时放于SettingsSystem中处理，接入了手势位置识别接口和语音识别接口，结合事件系统可以做全局通知。



### JarvenGameTool

JarvenGameTool里面包括响应式编程包Unirx和高性能异步库Unitask以及跑路人封装的功能框架，包括了有限状态机模板、单例模板、资源管理器、对象池、常用数据结构扩展等。

#### 单例
```c#
using JarvenFramework;

public MyClass : Singleton<MyClass> 
{
    public int MyValue = 1;
    public int MyMethod(int a, int b) 
    {
        return a + b;
    }
}
```
在外部中即可这样调用该单例
```c#
MyClass.Instance.MyValue = 2;
int i = MyClass.Instacne.MyMethon(1, 2);
```

#### 资源管理模块
基于原生addressable系统的资源管理系统，`请务必在拉取本工程后安装addressable`，在unity工程中找到```Window/package Manager```，左上角选取Packages:Unity Registry，右上角搜索Addressables安装即可。

使用方法也较为简单，点击需要流式加载的预制体，inspector中右上角勾选aa，之后会自动出现其在工程中的路径，在代码中引用即可。
例如想加载一个路径为`Assets/cube.prefabs`预制体模型，注意资源加载到内存后还需要实例化。
```c#
using JarvenFramework.ResModule;

// 同步方式加载
GameObject cube = GameObject.Instantiate( ResManager.Instance.LoadAsset<GameObject>("Assets/cube.prefabs"));

// 异步方式加载
ResManager.Instance.LoadAssetAsync<GameObject>("Assets/cube.prefabs", obj => {
    GameObject cube = GameObject.Instantiate(obj);
}); 
```


#### Unirx简单使用
响应式编程，主打一个一目了然，假设有一个外部单例方法MyClass.Instance.MyMethod();

```c#
using Unirx;

// 下一帧执行方法
Observable.NextFrame().Subscribe(_ => {
    MyClass.Instance.MyMethod();
});

// 每一点五秒执行一次该方法
Observable.Interval(TimeSpan.FromMilliseconds(50)).Sample(TimeSpan.FromSeconds(1.5f)).Subscribe(_ => {
    MyClass.Instance.MyMethod();     
});

// 对一个每帧执行的响应任务进行创建与销毁
private IDisposable MyTask;
MyTask = Observable.EveryUpdate().Subscribe(_ => {
    MyClass.Instance.MyMethod();     
});

MyTask.dispose();
```
