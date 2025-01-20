package frc.robot.subsystems.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

public class UdpListenerThread extends Thread {
  private final int port;
  private final int mtu;

  public UdpListenerThread(int port, int mtu) {
    this.port = port;
    this.mtu = mtu;

    setName("Vision UDP Listener [" + port + "]");
    setDaemon(true);
  }

  @Override
  public void run() {
    DatagramSocket socket = null;
    try {
      socket = new DatagramSocket(port);
      final var buffer = new byte[mtu];
      final var packet = new DatagramPacket(buffer, buffer.length);

      while (true) {
        // blocks until next datagram is received
        socket.receive(packet);
        packet.getData(); // do something with the data
      }
    } catch (Exception e) {
      // TODO: log exception
      e.printStackTrace();
    } finally {
      if (socket != null) {
        socket.close();
      }
    }
  }
}
