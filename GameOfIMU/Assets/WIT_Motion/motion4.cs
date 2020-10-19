using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;


public class motion4 : MonoBehaviour
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
        UDPrecv = new UdpClient(new IPEndPoint(IPAddress.Any, 2003));
        endpoint = new IPEndPoint(IPAddress.Any, 0);
        recvThread = new Thread(new ThreadStart(RecvThread));
        recvThread.IsBackground = true;
        recvThread.Start();
    }

    void FixedUpdate()
    {

        Quaternion dq = quaternion * Quaternion.Inverse(q_ref);

        Quaternion pos = Quaternion.Inverse(qv) * ( dq * qv );

        this.transform.localRotation = Quaternion.Lerp(this.transform.localRotation, pos, Time.deltaTime * 10);   
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

        //Debug.Log(quaternion);

        if(cail == 0) {
            q_ref = quaternion;

            cail = 1;

            Quaternion Q_virtual_creator = quaternion;

	        Vector3 world_X = new Vector3(1,0,0);
	        Vector3 world_Z = new Vector3(0,0,1);

	        Vector3 vc_Z         = Q_virtual_creator * world_Z;            // Apply rotation to Z axis of the sensor
	        Vector3 vc_Z_proy_XY = vc_Z - ( Vector3.Dot(vc_Z, world_Z) * world_Z ); // Projection: rotated Y axis on the XY plane
	        Vector3 forward_line = Vector3.Normalize(vc_Z_proy_XY);              // Normalize projection to create a "forward line"

	        float dotx = Vector3.Dot(world_X, forward_line);
	        float detx = Vector3.Dot(world_Z,Vector3.Cross(world_X,forward_line));
	        float angle = Mathf.Atan2(detx,dotx);							// Determine angle between w_X and the forward line

	        //AngleAxisd xToForward( angle, world_Z );
	        //Quaterniond Qvirtual( xToForward );								// Construct a quat with the needed rotation
            qv = Quaternion.AngleAxis(angle,Vector3.forward);
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
