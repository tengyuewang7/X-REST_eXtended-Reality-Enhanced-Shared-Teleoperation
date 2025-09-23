using JarvenFramework;

public class EventSystem : Singleton<EventSystem>, IGameModule
{
    private readonly EventManager<AppEventId> _appEvent = new EventManager<AppEventId>();
    public EventManager<AppEventId> AppEvent => _appEvent;

    public void Initialize()
    {

    }

    public void Release()
    {

    }
}