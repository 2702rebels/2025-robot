package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class GameConstants {
  /** Assumes robot bumpers are square. Outer width/height of the robot with bumpers on. */
  public static final double kBumperDimensions = Units.inchesToMeters(34);

  /** X coordinate (meters) of the robot center when at climb ready position. */
  public static final double kClimbPositionX = 8.2;

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
    final var gap = Units.inchesToMeters(8);
    final var gapL4 = Units.inchesToMeters(1);

    for (int i = 0; i < 12; ++i) {
      final var face = FieldConstants.Reef.faces[i / 2];
      final var offset =
          new Translation2d(kBumperDimensions / 2 + gap, branchShift * (i % 2 == 0 ? -1 : 1))
              .rotateBy(face.getRotation());

      final var offsetL4 =
          new Translation2d(kBumperDimensions / 2 + gapL4, branchShift * (i % 2 == 0 ? -1 : 1))
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

  /** Safety margin to use for safe field perimeter. */
  public static final double kFieldPerimeterSafetyMargin =
      kBumperDimensions / 2 + Units.inchesToMeters(12);

  /** Determines if the pose is at the desired climb position. */
  public static boolean isAtClimbPosition(Pose2d pose) {
    final var x = FieldConstants.fieldPose.apply(pose).getX();
    return MathUtil.isNear(kClimbPositionX, x, Units.inchesToMeters(1));
  }

  /** Determines if the pose is within safe field perimeter to be used for auto. */
  public static boolean withinAutoFieldPerimeter(Pose2d pose) {
    // here logic is defined for blue origin side, since we don't want to cross the center line,
    // flip the supplied pose if the robot is on the red alliance side
    final var adjustedPose = FieldConstants.fieldPose.apply(pose);

    final var x = adjustedPose.getX();
    final var y = adjustedPose.getY();
    final var maxX = FieldConstants.kStartingLineX.in(Meters);
    final var maxY = FieldConstants.kFieldWidth.in(Meters);

    if (x < kFieldPerimeterSafetyMargin
        || x > maxX - kFieldPerimeterSafetyMargin
        || y < kFieldPerimeterSafetyMargin
        || y > maxY - kFieldPerimeterSafetyMargin) {
      return false;
    }

    // account for loading station corners
    // Bottom: (0, 1.2m), (1.7m, 0)
    // LineEq: 0.7x + y - 1.2 < 0
    if (0.7 * x + y - 1.2 < kFieldPerimeterSafetyMargin
        || -0.7 * x + y + 1.2 - maxY > -kFieldPerimeterSafetyMargin) {
      return false;
    }

    return false;
  }
}
