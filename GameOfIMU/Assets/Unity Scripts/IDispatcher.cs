using System;
using System.Net;

public interface IDispatcher
{
    void Invoke(Action<IPEndPoint, byte[]> fn, IPEndPoint clientEndpoint, byte[] data);
}