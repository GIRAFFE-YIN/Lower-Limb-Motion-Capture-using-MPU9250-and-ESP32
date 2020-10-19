using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;

public class Dispatcher
{
    private List<Action<IPEndPoint, byte[]>> pending = new List<Action<IPEndPoint, byte[]>>();
    private List<IPEndPoint> pendingIPs = new List<IPEndPoint>();
    private List<byte[]> pendingData = new List<byte[]>();
    private static Dispatcher instance;

    // Schedule code for execution in the main-thread.
    public void Invoke(Action<IPEndPoint, byte[]> fn, IPEndPoint clientEndpoint, byte[] data)
    {
        lock (pending)
        {
            lock (pendingIPs)
            {
                lock (pendingData) 
                {
                    pending.Add(fn);
                    pendingIPs.Add(clientEndpoint);
                    pendingData.Add(data);
                }
            }
        }
    }

    // Called in the main thread to execute the pending code and clear the cue
    public void InvokePending()
    {
        lock (pending)
        {
            lock (pendingIPs)
            {
                lock (pendingData)
                {
                    for (int i = 0; i < pending.Count; i++)
                    {
                        pending[i](pendingIPs[i], pendingData[i]);
                    }

                    pending.Clear();
                    pendingIPs.Clear();
                    pendingData.Clear();
                }
            }
        }
    }

    // Instantiates a Dispatcher
    public static Dispatcher Instance
    {
        get
        {
            if (instance == null)
            {
                // Instance singleton on first use.
                instance = new Dispatcher();
            }
            return instance;
        }
    }
}