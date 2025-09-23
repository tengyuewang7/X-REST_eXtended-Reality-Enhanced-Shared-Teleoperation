using MessagePack;

[MessagePackObject]
public class MyClassGroup
{
    [Key(0)]
    public MyClass[] MyClasses { get; set; }
}
