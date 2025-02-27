package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = nt.getTable("DriveState");

  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

  private final StructPublisher<ChassisSpeeds> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();

  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  private final double[] poseArray = new double[3];
  private final double[] moduleStatesArray = new double[8];
  private final double[] moduleTargetsArray = new double[8];

  public Telemetry(boolean loggingEnabled) {
    if (loggingEnabled) {
      SignalLogger.start();
    }
  }

  /** Accepts the swerve drive state and log via SignalLogger and NT. */
  public void update(SwerveDriveState state) {
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    // signal logger
    poseArray[0] = state.Pose.getX();
    poseArray[1] = state.Pose.getY();
    poseArray[2] = state.Pose.getRotation().getDegrees();

    for (var i = 0; i < state.ModuleStates.length; ++i) {
      moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
      moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
  }
}
