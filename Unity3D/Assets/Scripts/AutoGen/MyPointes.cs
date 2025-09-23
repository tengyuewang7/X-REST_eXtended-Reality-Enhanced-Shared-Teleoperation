using MessagePack;

[MessagePackObject]
public class MyPointes
{
    [Key(0)]
    public MyPoint[] pointes { get; set; }
}
