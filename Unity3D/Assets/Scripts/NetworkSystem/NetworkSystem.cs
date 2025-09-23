/*using JarvenFramework;
using JarvenFramework.ResModule;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using UnityEngine;


public class NetworkSystem : Singleton<NetworkSystem>, IGameModule
{

    private Socket _clientSocket;
    private string _ipAddress;
    private int _port;
    private bool onHead = false;
    private int _cnt = 0;
    private int _nChunk = 0;

    private byte[] _buffer = new byte[1024 * 1024];
    private byte[] _cache;

    public void Initialize()
    {
        SetUpSocket();
    }

    public void Release()
    {
        ShutdownSocket();
    }

    public void StartConnect()
    {
        _clientSocket.BeginConnect(_ipAddress, _port, ConnectCallback, _clientSocket);
    }


    public void StartReceive()
    {
        for (int i= 0; i < _buffer.Length; i++)
        {
            _buffer[i] = 0;
        }
        Debug.Log($"Start Receive");
        _clientSocket.BeginReceive(_buffer, 0, _buffer.Length, SocketFlags.None, ReceiveCallback, null);
    }

    private void SetUpSocket()
    {
        GlobalSettingAsset asset = ResManager.Instance.LoadAsset<GlobalSettingAsset>("Assets/Scripts/GlobalSetting.asset");
        _ipAddress = asset.IpAddr;
        _port = asset.Port;

        _clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
    }

    private void ShutdownSocket()
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

    private void ReceiveCallback(IAsyncResult iAsyncResult)
    {
        try
        {
            *//*Socket socket = iAsyncResult.AsyncState as Socket;
            if (socket != null) {
                int len = socket.EndReceive(iAsyncResult);
                Debug.Log($"len: {len}");
                ParseInfo();
                StartReceive();
            }
            else
            {
                Debug.Log("socket is Null");
            }*//*

            int len = _clientSocket.EndReceive(iAsyncResult);
            Debug.Log($"len: {len}");
            if (len == 0)
            {
                return;
            }
            ParseInfo(len);
            if (!onHead)
            {
                _nChunk--;
                if (_nChunk == 0 )
                {
                    Debug.Log("all receive");
                    // todo parse
                }
            }
            
            StartReceive();
        }
        catch (SocketException ex)
        {
            Debug.LogError("Receive fail:" + ex.ToString());
        }
    }

    private void ParseInfo(int len)
    {
        byte[] cache = new byte[len];
        Array.Copy(_buffer, cache, len);

        if (onHead)
        {
            string info = InfoSystem.Instance.Deserialize<string>(cache);
            if (info.StartsWith("Genshin"))
            {
                string cut = info.Substring(len - 7);
                _nChunk = int.Parse(cut);
                onHead = false;
            }
        }
        else
        {
            List<float> test1 = InfoSystem.Instance.Deserialize<List<float>>(cache);
            SaveText(test1);
        }
    }


    private void SaveText(List<float> datas)
    {
        using (StreamWriter sw = new StreamWriter($"Assets/test{_cnt}.txt"))
        {
            _cnt++;
            foreach (var data in datas)
            {
                sw.WriteLine(data);
            }

            sw.Close();
        }
    }
}
*/