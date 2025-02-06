package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final double minDistance = 0.2;
  private final double maxDistance = 0.8;

  protected Supplier<Pose2d> poseSupplier;
  private final BooleanSupplier stopCondition;
  private final CommandSwerveDrivetrain drive;
  private boolean running;
  private Translation2d setpointTranslation;

  private final ProfiledPIDController distanceController =
      new ProfiledPIDController(
          DriveConstants.PoseChasing.translationKp,
          DriveConstants.PoseChasing.translationKi,
          DriveConstants.PoseChasing.translationKd,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxSpeed.in(MetersPerSecond) * 0.8,
              DriveConstants.maxAcceleration.in(MetersPerSecondPerSecond) * 0.8),
          Constants.cyclePeriodSeconds);

  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          DriveConstants.PoseChasing.rotationKp,
          DriveConstants.PoseChasing.rotationKi,
          DriveConstants.PoseChasing.rotationKd,
          new TrapezoidProfile.Constraints(
              DriveConstants.maxAngularRate.in(RadiansPerSecond),
              DriveConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond)),
          Constants.cyclePeriodSeconds);

  private double distanceErrorAbs;
  private double rotationErrorAbs;

  public DriveToPose(CommandSwerveDrivetrain drive, Pose2d pose) {
    this(drive, () -> pose, null);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drive, Supplier<Pose2d> poseSupplier, BooleanSupplier stopCondition) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    this.stopCondition = stopCondition;
    addRequirements(drive);

    distanceController.setTolerance(Units.inchesToMeters(1));
    rotationController.setTolerance(Units.degreesToRadians(2));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    final var currentFieldVelocity = drive.getFieldSpeeds();
    final var currentPose = drive.getPose();
    final var desiredPose = poseSupplier.get();

    final var currentTranslation = currentPose.getTranslation();
    final var desiredTranslation = desiredPose.getTranslation();

    final var distance = currentTranslation.getDistance(desiredTranslation);
    final var angle = currentTranslation.minus(desiredTranslation).getAngle();

    // reset all controllers
    distanceController.reset(
        distance,
        Math.min(
            0.0,
            -new Translation2d(
                    currentFieldVelocity.vxMetersPerSecond, currentFieldVelocity.vyMetersPerSecond)
                .rotateBy(angle.unaryMinus())
                .getX()));

    rotationController.reset(
        currentPose.getRotation().getRadians(), drive.getRobotSpeeds().omegaRadiansPerSecond);

    setpointTranslation = currentTranslation;
  }

  @Override
  public void execute() {
    running = true;

    final var currentPose = drive.getPose();
    final var desiredPose = poseSupplier.get();

    final var currentTranslation = currentPose.getTranslation();
    final var desiredTranslation = desiredPose.getTranslation();
    final var currentRotation = currentPose.getRotation().getRadians();
    final var desiredRotation = desiredPose.getRotation().getRadians();

    final var distance = currentTranslation.getDistance(desiredTranslation);
    final var angle = currentTranslation.minus(desiredTranslation).getAngle();

    distanceErrorAbs = distance;
    rotationErrorAbs = Math.abs(currentRotation - desiredRotation);

    final double scaleFF =
        MathUtil.clamp((distance - minDistance) / (maxDistance - minDistance), 0.0, 1.0);

    distanceController.reset(
        setpointTranslation.getDistance(desiredPose.getTranslation()),
        distanceController.getSetpoint().velocity);

    var driveVelocityScalar =
        distanceController.getSetpoint().velocity * scaleFF
            + distanceController.calculate(distance, 0.0);

    // stop driving when at goal
    if (distance < distanceController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    setpointTranslation =
        desiredTranslation.plus(
            new Translation2d(distanceController.getSetpoint().position, 0.0).rotateBy(angle));

    var rotationVelocity =
        rotationController.getSetpoint().velocity
            + rotationController.calculate(currentRotation, desiredRotation);

    // stop rotating when at goal
    if (rotationErrorAbs < rotationController.getPositionTolerance()) {
      rotationVelocity = 0.0;
    }

    final var driveVelocity = new Translation2d(driveVelocityScalar, 0.0).rotateBy(angle);

    drive.driveAt(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            rotationVelocity,
            currentPose.getRotation()));

    final var setpointPose =
        new Pose2d(setpointTranslation, new Rotation2d(distanceController.getSetpoint().position));

    DogLog.log("DriveToPose/Setpoint", setpointPose);
    DogLog.log("DriveToPose/Goal", desiredPose);
    DogLog.log("DriveToPose/DistanceMeasured", distance);
    DogLog.log("DriveToPose/DistanceSetpoint", distanceController.getSetpoint().position);
    DogLog.log("DriveToPose/DistanceVelocity", distanceController.getSetpoint().velocity);
    DogLog.log("DriveToPose/RotationMeasured", currentRotation);
    DogLog.log("DriveToPose/RotationSetpoint", rotationController.getSetpoint().position);
    DogLog.log("DriveToPose/RotationVelocity", rotationController.getSetpoint().velocity);
    DogLog.log("DriveToPose/RotationError", rotationErrorAbs);
    DogLog.log("DriveToPose/DistanceAtGoal", distanceController.atGoal());
    DogLog.log("DriveToPose/RotationAtGoal", rotationController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return atGoal() || (stopCondition != null && stopCondition.getAsBoolean());
  }

  public boolean atGoal() {
    return running && distanceController.atGoal() && rotationController.atGoal();
  }

  /** Checks if the robot pose is within the allowed translation and rotation tolerances. */
  public boolean withinTolerance(double distanceTolerance, Rotation2d rotationTolerance) {
    return running
        && distanceErrorAbs < distanceTolerance
        && rotationErrorAbs < rotationTolerance.getRadians();
  }

  public boolean isRunning() {
    return running;
  }
}
