package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.lib.ObjectDetectionPacket;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.UdpListenerThread;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class ObjectDetector {
  private final Lock lock = new ReentrantLock();
  private final UdpListenerThread<ObjectDetectionPacket> listener;
  private AtomicLong latency = new AtomicLong(0);

  // timestamped log of recently detected objects
  private final TimestampedCircularBuffer<Translation2d> detectedObjects =
      new TimestampedCircularBuffer<>(50);

  public ObjectDetector() {
    listener =
        new UdpListenerThread<ObjectDetectionPacket>(
            Constants.Network.UDP_PORT_OBJECT_DETECTOR,
            1500,
            this::packetReceived,
            ObjectDetectionPacket.struct::unpack);

    listener.start();
  }

  private void packetReceived(ObjectDetectionPacket packet) {
    if (packet.count > 0) {
      final var t = RobotController.getFPGATime();
      latency.set(t - packet.timestamp);

      // TODO: decide on which object(s) to take when multiple are detected
      final var o = packet.objects[0];
      lock.lock();
      try {
        detectedObjects.enqueue(
            packet.timestamp,
            new Translation2d(o.distance, 0).rotateBy(Rotation2d.fromRadians(o.angle)));
      } finally {
        lock.unlock();
      }
    }
  }

  /** Returns recent latency in microseconds. */
  @Logged(name = "Latency", importance = Importance.DEBUG)
  public long getLatency() {
    return latency.get();
  }
}
