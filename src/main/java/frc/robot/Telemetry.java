package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.generated.TunerConstants;

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

  private final DoubleArrayPublisher driveModuleSlipRatios =
      driveStateTable.getDoubleArrayTopic("ModuleSlipRatios").publish();

  private final DoubleArrayPublisher driveModuleSlipAccums =
      driveStateTable.getDoubleArrayTopic("ModuleSlipAccums").publish();

  private final DoubleArrayPublisher driveModuleTranslationVelocities =
      driveStateTable.getDoubleArrayTopic("ModuleTranslationVelocities").publish();

  private final DoubleArrayPublisher driveModuleVelocityDiff =
      driveStateTable.getDoubleArrayTopic("ModuleVelocityDiff").publish();

  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();

  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  private final double[] poseArray = new double[3];
  private final double[] moduleStatesArray = new double[8];
  private final double[] moduleTargetsArray = new double[8];
  private final double[] moduleSlipRatiosArray = new double[4];
  private final double[] moduleSlipAccumsArray = new double[4];
  private final double[] moduleVelocityDiffArray = new double[4];

  private final double[] moduleTranslationVelocities = new double[10];

  public Telemetry(boolean loggingEnabled) {
    if (loggingEnabled) {
      SignalLogger.setPath("/u/logs/ctre/");
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

    // detect slippage by comparing translation velocities of the wheels with IMU velocity
    final var robotVelocity =
        new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);

    final var robotVelocityNorm = robotVelocity.getNorm();
    moduleTranslationVelocities[0] = robotVelocity.getX();
    moduleTranslationVelocities[1] = robotVelocity.getY();

    for (var i = 0; i < state.ModuleStates.length; ++i) {
      final var moduleVelocity =
          new Translation2d(
              state.ModuleStates[i].speedMetersPerSecond, state.ModuleStates[i].angle);

      final var moduleTranslationVelocity =
          new Translation2d(
              moduleVelocity.getX()
                  + state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getY(),
              moduleVelocity.getY()
                  - state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getX());

      moduleTranslationVelocities[i * 2 + 2] = moduleTranslationVelocity.getX();
      moduleTranslationVelocities[i * 2 + 3] = moduleTranslationVelocity.getY();

      // robot can be stationary, but the slippage can still occur if we are trying to push against
      // an obstacle
      if (MathUtil.isNear(0, robotVelocityNorm, 0.3)) {
        moduleSlipRatiosArray[i] =
            MathUtil.isNear(0, moduleTranslationVelocity.getNorm(), 0.3) ? 0 : 1;
      } else {
        // technically we should compute projections on the robot IMU velocity vector
        // but all these vectors should be colinear unless the robot is falling apart
        moduleSlipRatiosArray[i] =
            (moduleTranslationVelocity.getNorm() - robotVelocityNorm) / robotVelocityNorm;
      }

      moduleSlipAccumsArray[i] += moduleSlipRatiosArray[i];

      moduleVelocityDiffArray[i] =
          state.ModuleTargets[i].speedMetersPerSecond - state.ModuleStates[i].speedMetersPerSecond;
    }

    driveModuleSlipRatios.set(moduleSlipRatiosArray);
    driveModuleSlipAccums.set(moduleSlipAccumsArray);
    driveModuleTranslationVelocities.set(moduleTranslationVelocities);
    driveModuleVelocityDiff.set(moduleVelocityDiffArray);

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
    SignalLogger.writeDoubleArray("DriveState/ModuleSlipRatios", moduleSlipRatiosArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
  }
}
