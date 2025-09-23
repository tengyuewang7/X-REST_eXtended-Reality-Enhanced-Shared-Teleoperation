using JarvenFramework;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using UnityEngine;
public class FtpSystem : Singleton<FtpSystem>, IGameModule
{

    private string remoteServerIp;
    private string remoteCatalog;
    private string remoteUser;
    private string remotePassword;
    private NetworkCredential ftpCredential;
    private string ftpUri;

    public void Initialize()
    {
        remoteServerIp = SettingsSystem.Instance.Asset.FtpIp;
        remoteCatalog = SettingsSystem.Instance.Asset.FtpCatalog;
        remoteUser = SettingsSystem.Instance.Asset.FtpUser;
        remotePassword = SettingsSystem.Instance.Asset.FtpPassword;

        ftpUri = $"ftp://{remoteServerIp}/{remoteCatalog}";
        ftpCredential = new NetworkCredential(remoteUser, remotePassword);
    }

    public void Release()
    {
        
    }

    public void Download(string localFileName, string ftpFileName)
    {
        try
        {
            string tmpFileName = $"{localFileName}.temp";

            using (FileStream fs = new FileStream(tmpFileName, FileMode.OpenOrCreate, FileAccess.ReadWrite))
            {
                FtpWebRequest req = (FtpWebRequest)WebRequest.Create(new Uri(ftpUri + ftpFileName));
                req.Credentials = ftpCredential;
                req.Proxy = null;
                req.UseBinary = true;
                req.Method = WebRequestMethods.Ftp.DownloadFile;
                FtpWebResponse resp = (FtpWebResponse)req.GetResponse();
                Stream ftpStream = resp.GetResponseStream();

                byte[] buffer = new byte[1024 * 1024 * 4];
                int read_cnt = ftpStream.Read(buffer, 0, buffer.Length);
                while (read_cnt > 0)
                {
                    fs.Write(buffer, 0, read_cnt);
                    read_cnt = ftpStream.Read(buffer, 0, buffer.Length);
                }
                ftpStream.Close();
                fs.Close();
                resp.Close();
            }

            if (File.Exists(localFileName))
            {
                File.Delete(localFileName);
            }
            File.Move(tmpFileName, localFileName);
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
        }
    }

    private int GetDownloadSize(string ftpFileName)
    {
        try
        {
            FtpWebRequest req = (FtpWebRequest)WebRequest.Create(new Uri(ftpUri + ftpFileName));
            req.Credentials = ftpCredential;
            req.Proxy = null;
            req.UseBinary = true;
            req.Method = WebRequestMethods.Ftp.GetFileSize;

            using (FtpWebResponse resp = (FtpWebResponse)req.GetResponse())
            {
                int downloadSize = (int)resp.ContentLength;
                return downloadSize;
            }
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
            return -1;
        }
    }
    /// <summary>
    /// 更新目标文件数据流，目前流读取有点问题，先弃用
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="ftpFileName"></param>
    private void UpdateStream<T>(string ftpFileName)
    {
        try
        {
            int len = GetDownloadSize(ftpFileName);
            FtpWebRequest req = (FtpWebRequest)WebRequest.Create(new Uri(ftpUri + ftpFileName));
            req.Credentials = ftpCredential;
            req.Proxy = null;
            req.UseBinary = true;
            req.Method = WebRequestMethods.Ftp.DownloadFile;


            using (FtpWebResponse resp = (FtpWebResponse)req.GetResponse())
            {
                using (Stream ftpStream = resp.GetResponseStream())
                {
                    byte[] bytes = new byte[len];
                    int read_cnt = ftpStream.Read(bytes, 0, bytes.Length);
                    EventSystem.Instance.AppEvent.Dispatch(AppEventId.ON_STREAM_UPDATE, InfoSystem.Instance.Deserialize<T>(bytes));
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
        }
    }
}
