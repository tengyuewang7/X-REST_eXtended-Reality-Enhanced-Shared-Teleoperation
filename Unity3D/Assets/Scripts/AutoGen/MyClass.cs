using MessagePack;

[MessagePackObject]
public class MyClass
{
    [Key(0)]
    public double Id { get; set; }
    [Key(1)]
    public bool IsActive { get; set; }
    [Key(2)]
    public string Name { get; set; }
}
