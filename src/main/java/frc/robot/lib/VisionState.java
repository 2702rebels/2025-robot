package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionState {
  /** Robot's pose in the field frame of reference */
  public final TimestampedValue<Pose2d> pose;

  /** AprilTag directions in the robot frame of reference */
  public final TimestampedTranslation2d[] tags;

  public VisionState(TimestampedValue<Pose2d> pose, TimestampedTranslation2d[] tags) {
    this.pose = pose;
    this.tags = tags;
  }
}
