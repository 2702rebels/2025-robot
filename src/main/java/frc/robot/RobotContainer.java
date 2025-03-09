// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.Intent;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.AprilTagDetector;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.ObjectDetector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Position;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private static final double maxSpeedMetersPerSecond = DriveConstants.maxSpeed.in(MetersPerSecond);
  private static final double maxAngularRateRadiansPerSecond =
      DriveConstants.maxAngularRate.in(RadiansPerSecond);

  private static double kControllerDeadband = 0; // e.g. 0.05 for 5%

  // setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeedMetersPerSecond * kControllerDeadband)
          .withRotationalDeadband(maxAngularRateRadiansPerSecond * kControllerDeadband)
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // use open-loop control for drive motors

  private final RobotState state;

  private final Telemetry logger = new Telemetry(Constants.Settings.loggingEnabled);

  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged(name = "Elevator", importance = Importance.INFO)
  private final Elevator elevator =
      new Elevator(Constants.CanID.MOTOR_ELEVATOR_LEADER, Constants.CanID.MOTOR_ELEVATOR_FOLLOWER);

  @Logged(name = "Pivot", importance = Importance.INFO)
  private final Pivot pivot =
      new Pivot(Constants.CanID.MOTOR_PIVOT_LEADER, Constants.CanID.MOTOR_PIVOT_FOLLOWER);

  @Logged(name = "Manipulator", importance = Importance.INFO)
  private final Manipulator manipulator =
      new Manipulator(
          Constants.CanID.MOTOR_MANIPULATOR_GUIDE, Constants.CanID.MOTOR_MANIPULATOR_CORAL);

  @Logged(name = "Superstructure", importance = Importance.INFO)
  private final Superstructure superstructure = new Superstructure(elevator, pivot, manipulator);

  @Logged(name = "ObjectDetector", importance = Importance.INFO)
  private final ObjectDetector objectDetector = new ObjectDetector();

  @Logged(name = "AprilTagDetector", importance = Importance.INFO)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  // startup validation checklist
  @Logged(name = "Checklist/PivotEncoder", importance = Importance.CRITICAL)
  private boolean pivotEncoderValid = false;

  // path follower
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  // slows down max rotation speed for more precise movements
  private final Trigger precisionRotation = controllerDriver.leftStick();

  public RobotContainer(RobotState state) {
    this.state = state;

    // bind internal drivetrain buffer of historical poses
    state.setHistoricalPoseSupplier(drivetrain::getPoseAt);

    // bind vision state supplier
    state.setVisionStateSupplier(aprilTagDetector::getState);

    // configure DogLog logging
    if (Constants.Settings.loggingEnabled) {
      DogLog.setOptions(new DogLogOptions().withCaptureConsole(true).withLogExtras(true));

      // log metadata
      DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
      DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
      DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
      DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
      DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);
    } else {
      DogLog.setEnabled(false);
    }

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    // autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    autoChooser.addRoutine(
        "DriveForward",
        () -> autoRoutines.naive("Drive forward", driveForward(Inches.of(24)).withTimeout(5)));

    autoChooser.addRoutine(
        "Score L2 (HG)", () -> autoRoutines.scoreL4_HG(driveForward(Inches.of(36)).withTimeout(5)));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureTriggers();
    configureDriverBindings();
  }

  /** Smooth out control input by squaring. */
  private double smoothInput(double value) {
    final var v = MathUtil.clamp(value, -1, 1);
    return Math.signum(value) * v * v;
  }

  /** Use this method to configure driver controller bindings. */
  private void configureDriverBindings() {
    if (!Constants.Settings.drivingDisabled) {
      // note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // drivetrain will execute this command periodically
          drivetrain.applyRequest(
              () ->
                  drive
                      // drive forward with negative Y (forward)
                      .withVelocityX(
                          -smoothInput(controllerDriver.getLeftY()) * maxSpeedMetersPerSecond)
                      // drive left with negative X (left)
                      .withVelocityY(
                          -smoothInput(controllerDriver.getLeftX()) * maxSpeedMetersPerSecond)
                      // drive counterclockwise with negative X (left)
                      .withRotationalRate(
                          -smoothInput(controllerDriver.getRightX())
                              * maxAngularRateRadiansPerSecond
                              * (precisionRotation.getAsBoolean() ? preciseRotation.get() : 1))));
    }

    // reset the field-centric heading
    controllerDriver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // set brake position (X)
    controllerDriver.b().whileTrue(drivetrain.brake());

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      controllerDriver
          .back()
          .and(controllerDriver.y())
          .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      controllerDriver
          .back()
          .and(controllerDriver.x())
          .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      controllerDriver
          .start()
          .and(controllerDriver.y())
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      controllerDriver
          .start()
          .and(controllerDriver.x())
          .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    drivetrain.registerTelemetry(logger::update);

    controllerDriver.x().onTrue(state.setIntent(Intent.NONE));

    controllerDriver.leftTrigger().onTrue(state.setIntent(Intent.CORAL_FLOOR_INTAKE));
    controllerDriver.leftBumper().onTrue(state.setIntent(Intent.CORAL_STATION_INTAKE));

    // coral scoring
    controllerDriver
        .rightTrigger()
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreAuto.negate())
        .onTrue(state.setIntent(Intent.CORAL_SCORE_AUTO));

    controllerDriver
        .rightBumper()
        .and(superstructure.hasCoral)
        .onTrue(state.setIntent(Intent.CORAL_SCORE_L1));

    // algae scoring
    // Commented out since untested.
    // controllerDriver
    //     .rightTrigger()
    //     .and(superstructure.hasAlgae)
    //     .onTrue(state.setIntent(Intent.ALGAE_SCORE_NET));

    controllerDriver
        .rightBumper()
        .and(superstructure.hasAlgae)
        .onTrue(state.setIntent(Intent.ALGAE_EJECT));

    controllerDriver.povLeft().onTrue(state.setIntent(Intent.ALGAE_INTAKE_L2));
    controllerDriver.povRight().onTrue(state.setIntent(Intent.ALGAE_INTAKE_L3));

    controllerDriver.povDown().onTrue(state.setIntent(Intent.CORAL_EJECT));
    controllerDriver.povDown().onFalse(state.setIntent(Intent.NONE));

    controllerDriver.povUp().onTrue(state.setIntent(Intent.HOVER));
    controllerDriver.povUp().onFalse(state.setIntent(Intent.NONE));

    // temporary fix until we have limit switch mechanism in place
    controllerDriver
        .y()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setAlgaePresence(!superstructure.hasAlgae.getAsBoolean())));

    // controllerDriver.povLeft().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
    // controllerDriver.povRight().whileTrue(new WheelDiameterCharacterization(drivetrain, -1));
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // intent-driven triggers

    // eject coral (emergency mechanism does not check for coral presence)
    state.shouldEjectCoral.and(superstructure.canEject).whileTrue(manipulator.ejectCoral());
    state.shouldEjectCoral.onTrue(superstructure.setPosition(Position.L1));

    // intake algae
    state
        .shouldIntakeAlgae
        .and(superstructure.hasAlgae.negate())
        .and(superstructure.canAlgaeIntake)
        .whileTrue(manipulator.intakeAlgae());

    state
        .shouldIntakeAlgaeL2
        .and(superstructure.hasAlgae.negate())
        .onTrue(superstructure.setPosition(Position.L2_ALGAE));

    state
        .shouldIntakeAlgaeL3
        .and(superstructure.hasAlgae.negate())
        .onTrue(superstructure.setPosition(Position.L3_ALGAE));

    // hold algae while presence is set unless ejecting
    superstructure
        .hasAlgae
        .and(state.shouldEjectAlgae.negate())
        .and(superstructure.canEjectAlgae.negate().or(superstructure.canScoreAlgae.negate()))
        .whileTrue(manipulator.holdAlgae());

    // eject algae
    state
        .shouldEjectAlgae
        .and(superstructure.hasAlgae)
        .onTrue(superstructure.setPosition(Position.EJECT_ALGAE));

    state
        .shouldEjectAlgae
        .and(superstructure.hasAlgae)
        .and(superstructure.canEjectAlgae)
        .onTrue(ejectAlgaeSequence());

    // score algae (net)
    state
        .shouldScoreAlgaeNet
        .and(superstructure.hasAlgae)
        .onTrue(superstructure.setPosition(Position.NET));

    state
        .shouldScoreAlgaeNet
        .and(superstructure.hasAlgae)
        .and(superstructure.canScoreAlgae)
        .onTrue(ejectAlgaeSequence());

    // intake coral from the floor
    state
        .shouldIntakeCoralFloor
        .and(superstructure.hasCoral.negate())
        .onTrue(superstructure.setPosition(Position.FLOOR));

    state
        .shouldIntakeCoralFloor
        .and(superstructure.hasCoral.negate())
        .and(superstructure.canFloorIntake)
        .whileTrue(manipulator.intakeCoral());

    // intake coral from the station
    state
        .shouldIntakeCoralStation
        .and(superstructure.hasCoral.negate())
        .onTrue(superstructure.setPosition(Position.STATION));

    state
        .shouldIntakeCoralStation
        .and(superstructure.hasCoral.negate())
        .and(superstructure.canStationIntake)
        .whileTrue(manipulator.intakeCoral());

    // reset intent once coral has been acquired
    state.shouldIntakeCoral.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.NONE));

    // rumble once the coral has been acquired
    superstructure.hasCoral.onTrue(rumble(Milliseconds.of(300)));

    // score coral at L1
    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L1));

    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL1)
        .and(controllerDriver.rightBumper().or(() -> DriverStation.isAutonomous()))
        .whileTrue(manipulator.ejectCoral());

    // score coral via auto target
    state.hasScoringPose.onTrue(new DriveToPose(drivetrain, () -> state.getScoringPose()));

    // brake to prevent robot backing when operating elevator/pivot
    // for scoring at L2 or higher, but only until we reach the position
    state
        .shouldScoreCoralL23
        .or(state.shouldScoreCoralL4)
        .and(superstructure.hasCoral)
        .and(state.scoringReady)
        .onTrue(superstructure.setAutoScoringPosition())
        .onTrue(drivetrain.brake().until(superstructure.canScoreAuto));

    // scoring L2, L3
    state
        .shouldScoreCoralL23
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreAuto)
        .and(controllerDriver.rightTrigger().or(() -> DriverStation.isAutonomous()))
        .whileTrue(manipulator.ejectCoral());

    // scoring L4
    state
        .shouldScoreCoralL4
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL4)
        .onTrue(
            state
                .resetScoringPose()
                .andThen(
                    driveForward(Inches.of(13))
                        .withTimeout(1.5)
                        .andThen(state.setIntent(Intent.CORAL_SCORE_L4_FINAL))));

    state
        .shoulScoreCoralL4Final
        .and(superstructure.hasCoral)
        .and(controllerDriver.rightTrigger())
        .onTrue(
            superstructure
                .maintainPosition(Position.L4_SCORED)
                .until(superstructure::isDoneScoringL4)
                .andThen(driveForward(Inches.of(-10)).withTimeout(1))
                .andThen(state.setIntent(Intent.NONE)));

    // reset intent once coral has been scored
    state
        .shouldScoreCoralL1
        .or(state.shouldScoreCoralL23)
        .and(superstructure.hasCoral.negate())
        .onTrue(state.setIntent(Intent.NONE));

    // hover
    state.shouldHover.onTrue(superstructure.setPosition(Position.HOVER));

    // [default behavior] stow when no intent is specified
    //
    // special handling for situations when elevator is above 54", e.g.
    // when we are stuck on the reef while trying to score L4 and are trying
    // to abort -- first go up, then back up, before stowing the pivot
    state.idle.whileTrue(
        Commands.either(
            Commands.sequence(
                superstructure.setHeightOverride(60),
                driveForward(Inches.of(-10)).withTimeout(1.5),
                superstructure.resetHeightOverride(),
                superstructure.setPosition(Position.STOWED)),
            superstructure.setPosition(Position.STOWED),
            () -> elevator.isAbovePosition(52)));

    // endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(rumble(Milliseconds.of(500)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200))));
  }

  /** Commands drive forward fixed distance. */
  private Command driveForward(Distance distance) {
    return new InstantCommand(
            () -> {
              final var pose = drivetrain.getPose();
              final var state = RobotState.getInstance();
              state.setTargetPose(
                  new Pose2d(
                      pose.getTranslation()
                          .plus(
                              new Translation2d(distance.in(Meters), 0)
                                  .rotateBy(pose.getRotation())),
                      pose.getRotation()));
            })
        .andThen(new DriveToPose(drivetrain, state::getTargetPose));
  }

  /** Command sequence to eject algae and reset intent and presence. */
  private Command ejectAlgaeSequence() {
    return manipulator
        .ejectAlgae()
        .withTimeout(1)
        .andThen(manipulator.stop())
        .andThen(state.setIntent(Intent.NONE))
        .andThen(new InstantCommand(() -> superstructure.setAlgaePresence(false)));
  }

  public Command getAutonomousCommand() {
    // run the routine selected from the auto chooser
    return autoChooser.selectedCommand();
  }

  /** Runs validation checklist at the startup. Must be invoked from the robotInit(). */
  public void initValidation() {
    // validate that pivot absolute encoder reading matches the expected angle
    // of the mechanism in its starting configuration; this ensure that encoder
    // is properly zeroed at the start of the match
    pivotEncoderValid = pivot.isAtAngle(Constants.initialPivotAngle);

    final var failed = !pivotEncoderValid;

    if (failed) {
      DogLog.logFault("Startup checklist failed", AlertType.kError);
    }
  }

  /** Creates a command to rumble the controllers. */
  private Command rumble(Time time) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
        Commands.waitTime(time),
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  // spotless:off
  private static final Tunable.DoubleValue endgameAlert1 = Tunable.value("Endgame/Alert1", 30.0);
  private static final Tunable.DoubleValue endgameAlert2 = Tunable.value("Endgame/Alert2", 10.0);
  private static final Tunable.DoubleValue preciseRotation = Tunable.value("Control/PreciseRotation", 0.2);
  // spotless:on
}
