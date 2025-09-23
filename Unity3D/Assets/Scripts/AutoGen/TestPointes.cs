using MessagePack;
using System.Collections.Generic;

[MessagePackObject]
public class TestPoint
{
    [Key(0)]
    public string name { get; set; }
    [Key(1)]
    public List<double> pointes { get; set; }
}
