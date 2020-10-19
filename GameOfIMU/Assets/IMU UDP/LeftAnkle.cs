using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using UnityEngine.Networking;


public class LeftAnkle : MonoBehaviour
{

    //UDP，使用UdpClient的基本对象
    //messageReceive用于在“异步接收”操作中标识是否已接收消息。
    //UDPrecv用于定义UdpClient对象。
    //endpoint用于存储要进行广播的IP地址及端口
    private bool messageReceive;
    private UdpClient UDPrecv;
    private IPEndPoint endpoint;


    //UDP接收
    //recvBuf[] 为接收到的消息
    //recvThread 为接收线程
    private byte[] recvBuf;
    private Thread recvThread;

    float qx, qy, qz, qw;

    void Start()
    {
        UDPrecv = new UdpClient(new IPEndPoint(IPAddress.Any, 3333));
        endpoint = new IPEndPoint(IPAddress.Any, 0);


        recvThread = new Thread(new ThreadStart(RecvThread));
        recvThread.IsBackground = true;
        recvThread.Start();
    }

    void Update()
    {

        this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation, new Quaternion(qw, qz, qy, qx), Time.deltaTime * 10);
    }


    private void ReceiveCallback(IAsyncResult ar)
    {
        byte[] recvBuf = UDPrecv.EndReceive(ar, ref endpoint);

        qw = System.BitConverter.ToSingle(recvBuf, 0);
        qx = System.BitConverter.ToSingle(recvBuf, 8);
        qy = System.BitConverter.ToSingle(recvBuf, 4);
        qz = System.BitConverter.ToSingle(recvBuf, 12);

        messageReceive = true;
        //Debug.Log(msg);
        //Debug.Log(qx);
    }

    private void RecvThread()
    {
        messageReceive = true;
        while (true)
        {
            if (messageReceive)
            {
                UDPrecv.BeginReceive(new AsyncCallback(ReceiveCallback), null);
                messageReceive = false;
            }
            else
            {
                Thread.Sleep(100);
            }
        }
    }
}