using JarvenFramework;
using MessagePack;
using MessagePack.Resolvers;
using System.IO;

public class InfoSystem: Singleton<InfoSystem>, IGameModule
{
    public void Initialize()
    {
        StaticCompositeResolver.Instance.Register(
               GeneratedResolver.Instance,
               StandardResolver.Instance
          );

        var option = MessagePackSerializerOptions.Standard.WithResolver(StaticCompositeResolver.Instance);

        MessagePackSerializer.DefaultOptions = option;
    }

    public void Release()
    {
        
    }

    public T Deserialize<T>(byte[] data)
    {
        T info = MessagePackSerializer.Deserialize<T>(data);
        return info;
    }

    public T Deserialize<T>(string path)
    {
        using (FileStream fileStream = new FileStream(path, FileMode.Open, FileAccess.Read))
        {
            byte[] buffer = new byte[fileStream.Length];
            fileStream.Read(buffer, 0, buffer.Length);
            T info = MessagePackSerializer.Deserialize<T>(buffer);
            return info;
        }
    }

    public byte[] Serialize<T>(T data)
    {
       return MessagePackSerializer.Serialize(data);
    }

}