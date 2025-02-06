package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  private static final ChassisSpeeds ZeroSpeeds = new ChassisSpeeds();

  // Blue alliance sees forward as 0 degrees (toward red alliance wall)
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  // Red alliance sees forward as 180 degrees (toward blue alliance wall)
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  // keep track if we've ever applied the operator perspective before or not
  private boolean hasAppliedOperatorPerspective = false;

  // swerve request to apply during field-centric path following
  private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  // swerve request to apply during robot-centric path following
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  // PID controllers used with Choreo path following
  private final PIDController pathXController =
      new PIDController(
          DriveConstants.PathFollowing.translationKp,
          DriveConstants.PathFollowing.translationKi,
          DriveConstants.PathFollowing.translationKd,
          Constants.cyclePeriodSeconds);

  private final PIDController pathYController =
      new PIDController(
          DriveConstants.PathFollowing.translationKp,
          DriveConstants.PathFollowing.translationKi,
          DriveConstants.PathFollowing.translationKd,
          Constants.cyclePeriodSeconds);

  private final PIDController pathThetaController =
      new PIDController(
          DriveConstants.PathFollowing.rotationKp,
          DriveConstants.PathFollowing.rotationKi,
          DriveConstants.PathFollowing.rotationKd,
          Constants.cyclePeriodSeconds);

  // swerve requests to apply during SysId characterization
  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // SysId routine for characterizing translation (used to find PID gains for the drive motors)
  private final SysIdRoutine sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4V to prevent brownout
              null, // Use default timeout (10s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(translationCharacterization.withVolts(output)), null, this));

  // SysId routine for characterizing steer (used to find PID gains for the steer motors)
  private final SysIdRoutine sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1V/s)
              Volts.of(7), // Use dynamic voltage of 7V
              null, // Use default timeout (10s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(steerCharacterization.withVolts(volts)), null, this));

  // SysId routine for characterizing rotation (used to find PID gains for the
  // FieldCentricFacingAngle HeadingController)
  //
  // See docs of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId
  private final SysIdRoutine sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* Output is actually radians per second, but SysId only supports "volts" */
                setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* Also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  // SysId routine to use
  private SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;

  private void init() {
    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    init();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    init();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    init();
  }

  /**
   * Returns current robot's pose.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /**
   * Returns current robot-centric velocity.
   *
   * @return The current robot-centric chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return getState().Speeds;
  }

  /**
   * Returns current field-centric velocity.
   *
   * @return The current field-centric chassis speeds.
   */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), getState().Pose.getRotation());
  }

  /**
   * Creates a new auto factory for this drivetrain.
   *
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  /**
   * Creates a new auto factory for this drivetrain with the given trajectory logger.
   *
   * @param trajectoryLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajectoryLogger) {
    return new AutoFactory(
        this::getPose, this::resetPose, this::driveVia, true, this, trajectoryLogger);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Returns a command that follows the specified PP path.
   *
   * <p>See https://pathplanner.dev/pplib-create-a-path-on-the-fly.html for instructions on how to
   * generate path on the fly.
   *
   * @param path PP path to follow
   * @return Command to run
   */
  public Command followPathCommand(PathPlannerPath path) {
    try {
      return new FollowPathCommand(
          path,
          this::getPose,
          this::getRobotSpeeds,
          this::driveAt,
          new PPHolonomicDriveController(
              DriveConstants.PathFollowing.translationPID,
              DriveConstants.PathFollowing.rotationPID),
          DriveConstants.robotConfig,
          // DO NOT flip the path since this command is intended to be used
          // with on-the-fly generated trajectories
          () -> false,
          this);
    } catch (Exception e) {
      DriverStation.reportError("followPathCommand: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Follows the given field-centric (Choreo) path sample with PID.
   *
   * @param sample Sample along the path to follow
   */
  public void driveVia(SwerveSample sample) {
    final var pose = getPose();
    final var targetSpeeds = sample.getChassisSpeeds();

    targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    setControl(
        pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /**
   * Drives with the specified robot-centric speeds and drive feedforwards.
   *
   * <p>Use this version with PP commands.
   *
   * @param speeds robot-centric chassis speeds
   * @param ff PP drive feedforwards
   */
  public void driveAt(ChassisSpeeds speeds, DriveFeedforwards ff) {
    setControl(
        pathApplyRobotSpeeds
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(ff.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(ff.robotRelativeForcesYNewtons()));
  }

  /**
   * Drives with the specified robot-centric speeds.
   *
   * @param speeds robot-centric chassis speeds
   */
  public void driveAt(ChassisSpeeds speeds) {
    setControl(pathApplyRobotSpeeds.withSpeeds(speeds));
  }

  /** Stops the robot by setting speeds to zero. */
  public void stop() {
    driveAt(ZeroSpeeds);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    // periodically try to apply the operator perspective: if we haven't applied the operator
    // perspective before, then we should apply it regardless of DS state.
    // This allows us to correct the perspective in case the robot code restarts mid-match.
    // Otherwise, only check and apply the operator perspective if the DS is disabled.
    // This ensures driving behavior doesn't change until an explicit disable event occurs during
    // testing.
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    // run simulation at a faster rate so PID gains behave more reasonably
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              // use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
