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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.LedResource;
import frc.robot.subsystems.LedVision;
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
          .withDriveRequestType(DriveRequestType.Velocity);

  private final RobotState state = RobotState.getInstance();

  private final Telemetry logger = new Telemetry(Constants.Settings.loggingEnabled);

  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged(name = "Elevator", importance = Importance.INFO)
  private final Elevator elevator =
      new Elevator(
          Constants.CanivoreID.MOTOR_ELEVATOR_LEADER,
          Constants.CanivoreID.MOTOR_ELEVATOR_FOLLOWER,
          TunerConstants.kCANBus);

  @Logged(name = "Pivot", importance = Importance.INFO)
  private final Pivot pivot =
      new Pivot(Constants.CanID.MOTOR_PIVOT_LEADER, Constants.CanID.MOTOR_PIVOT_FOLLOWER);

  @Logged(name = "Manipulator", importance = Importance.INFO)
  private final Manipulator manipulator =
      new Manipulator(
          Constants.CanID.MOTOR_MANIPULATOR_GUIDE, Constants.CanID.MOTOR_MANIPULATOR_CORAL);

  @Logged(name = "Climb", importance = Importance.INFO)
  private final Climb climb = new Climb(Constants.CanID.MOTOR_CLIMB_WINCH);

  @Logged(name = "Superstructure", importance = Importance.INFO)
  private final Superstructure superstructure = new Superstructure(elevator, pivot, manipulator);

  @Logged(name = "ObjectDetector", importance = Importance.INFO)
  private final ObjectDetector objectDetector = new ObjectDetector();

  @Logged(name = "AprilTagDetector", importance = Importance.INFO)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  private final LedResource led = new LedResource(Constants.LedConstants.PWM_PORT);
  private final LedFeedback ledFeedback = new LedFeedback(led, this::getDefaultLedFeedback);
  private final LedVision ledVision = new LedVision(led, this::getDefaultVisionFeedback);

  // startup validation checklist
  @Logged(name = "Checklist/PivotEncoder", importance = Importance.CRITICAL)
  private boolean pivotEncoderValid = false;

  // path follower
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  // slows down max rotation speed for more precise movements
  private final Trigger precisionRotation = controllerDriver.leftStick();

  // button mappings
  private final Trigger resetOrientationButton = controllerDriver.start();
  private final Trigger brakeButton = controllerDriver.b();
  private final Trigger resetIntentButton = controllerDriver.x();
  private final Trigger floorIntakeButton = controllerDriver.leftTrigger();
  private final Trigger stationIntakeButton = controllerDriver.leftBumper();

  private final Trigger hoverButton = controllerDriver.povUp();
  private final Trigger coralEjectButton = controllerDriver.povDown();
  private final Trigger scoreButton = controllerDriver.rightTrigger();
  private final Trigger scoreManualButton = controllerDriver.rightBumper();

  // algae manipulation
  private final Trigger algaeIntakeL2Button = controllerDriver.povLeft();
  private final Trigger algaeIntakeL3Button = controllerDriver.povRight();
  private final Trigger toggleAlgaeButton = controllerDriver.y();

  private static final double kDriverTolerance = 0.05;

  // trigger for detecting driver input
  private final Trigger hasDriverInput =
      new Trigger(
          () ->
              !MathUtil.isNear(0, controllerDriver.getLeftX(), kDriverTolerance)
                  || !MathUtil.isNear(0, controllerDriver.getLeftY(), kDriverTolerance)
                  || !MathUtil.isNear(0, controllerDriver.getRightX(), kDriverTolerance));

  // trigger for scoring coral auto or manually
  private final Trigger shouldInitiateCoralSequence =
      state
          .isAuto
          .and(state.atScoringPose)
          .debounce(0.25)
          .or(scoreButton.debounce(0.5, DebounceType.kFalling));

  // slew rate limiter
  private final SlewRateLimiter velocityLimiter = new SlewRateLimiter(1, -5, 0);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(1, -5, 0);

  public RobotContainer() {
    // bind suppliers to the state
    state.setHistoricalPoseSupplier(drivetrain::getPoseAt);
    state.setVisionStateSupplier(aprilTagDetector::getState);
    state.setDetectedObjectSupplier(objectDetector::getDetectedObject);
    state.setDriveStateSupplier(drivetrain::getState);
    state.setOdometryPoseReset(drivetrain::resetPose);

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine(
        "DriveForward",
        () -> autoRoutines.naive("Drive forward", driveForward(Inches.of(24)).withTimeout(5)));

    autoChooser.addRoutine("Score L2", autoRoutines::scoreL2);
    autoChooser.addRoutine("Score L3", autoRoutines::scoreL3);
    autoChooser.addRoutine("Score L4", autoRoutines::scoreL4);
    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);

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
              () -> {
                // drive forward with negative Y (forward)
                var vx = -smoothInput(controllerDriver.getLeftY());
                // drive left with negative X (left)
                var vy = -smoothInput(controllerDriver.getLeftX());
                // drive counterclockwise with negative X (left)
                var w = -smoothInput(controllerDriver.getRightX());

                // apply rate limiter
                final var heading = Math.atan2(vx, vy);
                final var v = MathUtil.clamp(velocityLimiter.calculate(Math.hypot(vx, vy)), -1, 1);

                vx = v * Math.sin(heading);
                vy = v * Math.cos(heading);

                return drive
                    .withVelocityX(vx * maxSpeedMetersPerSecond)
                    .withVelocityY(vy * maxSpeedMetersPerSecond)
                    .withRotationalRate(
                        MathUtil.clamp(
                                Math.signum(w) * rotationLimiter.calculate(Math.abs(w)), -1, 1)
                            * maxAngularRateRadiansPerSecond
                            * (precisionRotation.getAsBoolean() ? preciseRotation.get() : 1));
              }));
    }

    resetOrientationButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    brakeButton.whileTrue(drivetrain.brake());

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      controllerDriver.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      controllerDriver.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      controllerDriver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      controllerDriver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      // controllerDriver.povLeft().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
      // controllerDriver.povRight().whileTrue(new WheelDiameterCharacterization(drivetrain, -1));
    }

    drivetrain.registerTelemetry(logger::update);

    resetIntentButton.onTrue(state.setIntent(Intent.kNone));

    // coral intake
    floorIntakeButton.onTrue(state.setIntent(Intent.kCoralFloorIntake));
    stationIntakeButton.onTrue(state.setIntent(Intent.kCoralStationIntake));

    // coral scoring
    scoreButton.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.kCoralScoreAuto));
    scoreManualButton.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.kCoralScoreL1));

    // algae scoring
    scoreButton.and(superstructure.hasAlgae).onTrue(state.setIntent(Intent.kAlgaeScoreNet));
    scoreManualButton.and(superstructure.hasAlgae).onTrue(state.setIntent(Intent.kAlgaeEject));

    algaeIntakeL2Button.onTrue(state.setIntent(Intent.kAlgaeIntakeL2));
    algaeIntakeL3Button.onTrue(state.setIntent(Intent.kAlgaeIntakeL3));

    coralEjectButton.onTrue(state.setIntent(Intent.kCoralEject));
    coralEjectButton.onFalse(state.setIntent(Intent.kNone));

    hoverButton.onTrue(state.setIntent(Intent.kHover));
    hoverButton.onFalse(state.setIntent(Intent.kNone));

    // manually toggle algae presence (until we have limit switch mechanism in place)
    toggleAlgaeButton.onTrue(
        new InstantCommand(
            () -> superstructure.setAlgaePresence(!superstructure.hasAlgae.getAsBoolean())));

    // controllerDriver.povLeft().onTrue(new InstantCommand(() -> climb.setWinchPosition(0),
    // climb));
    // controllerDriver
    //     .povRight()
    //     .onTrue(new InstantCommand(() -> climb.setWinchPosition(climbWinch.get()), climb));
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
        .and(superstructure.canIntakeAlgae)
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

    // state
    //     .shouldScoreAlgaeNet
    //     .and(superstructure.hasAlgae)
    //     .and(superstructure.canScoreAlgae)
    //     .onTrue(ejectAlgaeSequence());

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
    state.shouldIntakeCoral.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.kNone));

    // rumble once the coral has been acquired
    superstructure
        .hasCoral
        .onTrue(rumble(Milliseconds.of(300)))
        .onTrue(ledFeedback.signalSuccess());

    // score coral at L1
    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L1));

    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL1)
        .and(state.isAuto.or(scoreButton.debounce(0.5, DebounceType.kFalling)))
        .whileTrue(manipulator.ejectCoral());

    // score coral via auto target
    state
        .isAuto
        .and(state.hasScoringPose)
        .onTrue(new DriveToPose(drivetrain, () -> state.getScoringPose()).until(hasDriverInput));

    // reset auto mode once driver input is received
    state.isAuto.and(hasDriverInput).onTrue(state.resetAuto());

    // scoring L2, L3 (elevate to L2 and L3 as we are driving)
    state
        .shouldScoreCoralL2
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L2));

    state
        .shouldScoreCoralL2
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL2)
        .and(shouldInitiateCoralSequence)
        .whileTrue(manipulator.ejectCoral().andThen(state.markScoringTarget()));

    state
        .shouldScoreCoralL3
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L3));

    state
        .shouldScoreCoralL3
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL3)
        .and(shouldInitiateCoralSequence)
        .whileTrue(manipulator.ejectCoral().andThen(state.markScoringTarget()));

    // scoring L4
    state
        .shouldScoreCoralL4
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L4));

    state
        .shouldScoreCoralL4
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL4)
        .and(shouldInitiateCoralSequence)
        .onTrue(scoreL4Sequence());

    // reset intent once coral has been scored
    state
        .shouldScoreCoralL1
        .or(state.shouldScoreCoralL2)
        .or(state.shouldScoreCoralL3)
        .and(superstructure.hasCoral.negate())
        .onTrue(state.setIntent(Intent.kNone));

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

    // LED
    state.atScoringPose.whileTrue(ledFeedback.signalReady());
    state.isAuto.whileTrue(ledVision.signalAssist());

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

  /** Returns feedback for LED subsystem */
  private LedFeedback.Type getDefaultLedFeedback() {
    switch (state.intent) {
      case kCoralScoreL1:
      case kCoralScoreL2:
      case kCoralScoreL3:
      case kCoralScoreL4:
        return LedFeedback.Type.kScoringCoral;
      case kCoralFloorIntake:
      case kCoralStationIntake:
        return LedFeedback.Type.kIntakingCoral;
      case kAlgaeScoreNet:
      case kAlgaeEject:
        return LedFeedback.Type.kScoringAlgae;
      case kAlgaeIntakeL2:
      case kAlgaeIntakeL3:
        return LedFeedback.Type.kIntakingAlgae;
      default:
        if (superstructure.hasCoral.getAsBoolean()) {
          return LedFeedback.Type.kHoldingCoral;
        }
        if (superstructure.hasAlgae.getAsBoolean()) {
          return LedFeedback.Type.kHoldingAlgae;
        }
        return LedFeedback.Type.kNone;
    }
  }

  private LedVision.Type getDefaultVisionFeedback() {
    return LedVision.Type.kNone;
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
        .andThen(state.setIntent(Intent.kNone))
        .andThen(new InstantCommand(() -> superstructure.setAlgaePresence(false)));
  }

  /** Command sequence to score at L4 and reset. */
  private Command scoreL4Sequence() {
    return state
        .resetScoringPose()
        .andThen(
            Commands.sequence(
                driveForward(Inches.of(12)).withTimeout(1),
                state.setIntent(Intent.kCoralScoreL4Final),
                superstructure
                    .maintainPosition(Position.L4_SCORED)
                    .until(superstructure::isDoneScoringL4),
                state.markScoringTarget(),
                driveForward(Inches.of(-10)).withTimeout(1),
                state.setIntent(Intent.kNone)));
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
