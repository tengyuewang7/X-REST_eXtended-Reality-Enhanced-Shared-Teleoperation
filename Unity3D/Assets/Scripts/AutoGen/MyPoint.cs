using MessagePack;

[MessagePackObject]
public class MyPoint
{
    [Key(0)]
    public int name { get; set; }
    [Key(1)]
    public double x { get; set; }
    [Key(2)]
    public double y { get; set; }
    [Key(3)]
    public double z { get; set; }
}
