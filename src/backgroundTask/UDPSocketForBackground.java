package backgroundTask;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

public class UDPSocketForBackground extends Thread {

	private DatagramSocket ds; 
	
	public UDPSocketForBackground(String ip,int port) throws SocketException
	{
		ds = new DatagramSocket();
		ds.connect(new InetSocketAddress("172.31.1.148", 30003));
	}
	
	public void send(byte[] message) throws IOException
	{
		if(ds.isConnected())
		{
			DatagramPacket dp = new DatagramPacket(message, message.length);
			ds.send(dp);
		}
		else
		{
			ds.connect(new InetSocketAddress("172.31.1.148", 30003));
			ds.send(new DatagramPacket(message, message.length));
		}
	}
	
}
