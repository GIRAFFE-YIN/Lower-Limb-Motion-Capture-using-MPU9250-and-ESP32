using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;


public class motion8 : MonoBehaviour
{

    private bool messageReceive;
    private UdpClient UDPrecv;
    private IPEndPoint endpoint;

    private byte[] recvBuf;
    private Thread recvThread;

    float qx, qy, qz, qw;

    float angle_x, angle_y, angle_z;

    Quaternion quaternion = new Quaternion(1,0,0,0);
    Quaternion q_ref = new Quaternion(1,0,0,0);
    Quaternion qv = new Quaternion(1,0,0,0);
    Quaternion pos = new Quaternion(1,0,0,0);

    int cail = 0;

    void Start()
    {
        UDPrecv = new UdpClient(new IPEndPoint(IPAddress.Any, 2007));
        endpoint = new IPEndPoint(IPAddress.Any, 0);


        recvThread = new Thread(new ThreadStart(RecvThread));
        recvThread.IsBackground = true;
        recvThread.Start();
    }

    void FixedUpdate()
    {
        if(cail == 0) {
            qv = this.transform.localRotation;
            cail = 1;
        }

        Quaternion dq = quaternion * Quaternion.Inverse(q_ref);

        Quaternion pos = Quaternion.Inverse(qv) * ( dq * qv );

        this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation, pos, Time.deltaTime * 10);   

        Vector3 rotation = transform.rotation.eulerAngles;

        //this.transform.parent.transform.eulerAngles = new Vector3(0,angle_z,0);
        //Debug.Log(rotation);
        //Debug.Log(rotation[1]);


    }


    private void ReceiveCallback(IAsyncResult ar)
    {
        
        byte[] recvBuf = UDPrecv.EndReceive(ar, ref endpoint);

        string strWords = Encoding.UTF8.GetString(recvBuf);

        string[] values = strWords.Split(',');

        if (values.Length == 24) 
        {
            angle_x = float.Parse(values[6]);
            angle_y = float.Parse(values[7]);
            angle_z = float.Parse(values[8]);
            qw = float.Parse(values[16]);
            qx = float.Parse(values[17]);
            qy = float.Parse(values[18]);
            qz = float.Parse(values[19]);  
        }

        quaternion.w = qw;
        quaternion.x = qx;
        quaternion.y = -qz;
        quaternion.z = qy;

        if(cail == 1) {
            q_ref = quaternion;
            cail = 2;
        }

        messageReceive = true;
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
                Thread.Sleep(10);
            }
        }
    }
}
