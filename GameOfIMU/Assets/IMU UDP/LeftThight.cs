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


public class LeftThight : MonoBehaviour
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

    Quaternion quaternion = new Quaternion(1,0,0,0);

    Quaternion q3 = Quaternion.AngleAxis(-90,Vector3.right);  
    //q3.eulerAngles = new Vector3(90, 0, 0);
    
    //Quaternion pos = new Quaternion(1,0,0,0);


    void Start()
    {
        UDPrecv = new UdpClient(new IPEndPoint(IPAddress.Any, 2101));
        endpoint = new IPEndPoint(IPAddress.Any, 0);


        recvThread = new Thread(new ThreadStart(RecvThread));
        recvThread.IsBackground = true;
        recvThread.Start();
    }

    void FixedUpdate()
    {

        //transform.localRotation = Quaternion.Lerp(transform.localRotation, new Quaternion(qx, qy, qz, qw), Time.deltaTime * 1);
        //transform.eulerAngles = new Vector3(-angle_y, -angle_z, angle_x);
        //transform.eulerAngles = new Vector3(angle_x, -angle_z, angle_y);
        //transform.localRotation = quaternion;
        Quaternion pos =  q3 * quaternion;
        this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation, pos, Time.deltaTime * 7);
    }


    private void ReceiveCallback(IAsyncResult ar)
    {
        byte[] recvBuf = UDPrecv.EndReceive(ar, ref endpoint);

        string strWords = Encoding.UTF8.GetString(recvBuf);

        string[] values = strWords.Split(',');

        qw = float.Parse(values[0]);
        qx = float.Parse(values[1]);
        qy = float.Parse(values[2]);
        qz = float.Parse(values[3]);

        quaternion.w = qw;
        quaternion.x = qx;
        quaternion.y = qy;
        quaternion.z = qz;

        //Debug.Log(quaternion);

        messageReceive = true;
        //Debug.Log(msg);
        
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