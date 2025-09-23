using System;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;

public class TestSocket:MonoBehaviour
{
    private Socket _clientSocket;
    private byte[] _buffer = new byte[1024];
    private string _ipAddress = "192.168.101.101";
    private int _port = 4081;


    public void Start()
    {
        _clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        _clientSocket.BeginConnect(_ipAddress, _port, ConnectCallback, _clientSocket);
    }

    public void OnDestroy()
    {
        _clientSocket.Shutdown(SocketShutdown.Receive);
        _clientSocket.Close();
    }
    private void ConnectCallback(IAsyncResult iAsyncResult)
    {
        try
        {
            Socket socket = iAsyncResult.AsyncState as Socket;
            socket.EndConnect(iAsyncResult);
            Debug.Log("Connect sucessful");
            StartReceive();
        }
        catch (SocketException ex)
        {
            Debug.LogError("Connect fail:" + ex.ToString());
        }
    }

    private void StartReceive()
    {
        _clientSocket.BeginReceive(_buffer, 0, _buffer.Length, SocketFlags.None, ReceiveCallback, this);
    }

    private void ReceiveCallback(IAsyncResult iAsyncResult)
    {
        try
        {
            Socket socket = iAsyncResult.AsyncState as Socket;
            int len = socket.EndReceive(iAsyncResult);
            Debug.Log($"len{len}");
            ParseInfo(len);
            StartReceive();
        }
        catch (SocketException ex)
        {
            Debug.LogError("Connect fail:" + ex.ToString());
        }
    }
    private void ParseInfo(int len)
    {
        byte[] cache = new byte[len];
        Array.Copy(_buffer, cache, len);
        List<float> test1 = InfoSystem.Instance.Deserialize<List<float>>(cache);
        Debug.Log(test1.Count);
    }
}
