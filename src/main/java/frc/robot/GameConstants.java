package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class GameConstants {
  /** Assumes robot bumpers are square. Outer width/height of the robot with bumpers on. */
  public static final double bumperDimensions = Units.inchesToMeters(34);

  /**
   * Robot poses for scoring at each reef branch starting at the face closest to the fall in
   * counterclockwise order. Index corresponds to the A, B, ..., K, L references in the game manual.
   */
  public static final Pose2d[] reefScoringPoses = new Pose2d[12];

  /** Robot poses for scoring at L4 level (different gap). */
  public static final Pose2d[] reefScoringPosesL4 = new Pose2d[12];

  static {
    final var rotation180 = Rotation2d.fromDegrees(180);
    final var branchShift = Units.inchesToMeters(13) / 2;
    final var gap = Units.inchesToMeters(11);
    final var gapL4 = Units.inchesToMeters(11);

    for (int i = 0; i < 12; ++i) {
      final var face = FieldConstants.Reef.faces[i / 2];
      final var offset =
          new Translation2d(bumperDimensions / 2 + gap, branchShift * (i % 2 == 0 ? -1 : 1))
              .rotateBy(face.getRotation());

      final var offsetL4 =
          new Translation2d(bumperDimensions / 2 + gapL4, branchShift * (i % 2 == 0 ? -1 : 1))
              .rotateBy(face.getRotation());

      final var rotation = face.getRotation().rotateBy(rotation180);

      reefScoringPoses[i] = new Pose2d(face.getTranslation().plus(offset), rotation);
      reefScoringPosesL4[i] = new Pose2d(face.getTranslation().plus(offsetL4), rotation);

      // System.out.println(
      //     i
      //         + ": "
      //         + String.format("x=%.3f", reefScoringPoses[i].getX())
      //         + String.format(" y=%.3f", reefScoringPoses[i].getY())
      //         + String.format(" r=%.0f", reefScoringPoses[i].getRotation().getDegrees()));
    }
  }
}
