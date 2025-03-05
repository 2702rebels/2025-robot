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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.Intent;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Tunable;
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

  // setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeedMetersPerSecond * 0.05)
          .withRotationalDeadband(maxAngularRateRadiansPerSecond * 0.05) // add a 5% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // use open-loop control for drive motors

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

  // startup validation checklist
  @Logged(name = "Checklist/PivotEncoder", importance = Importance.CRITICAL)
  private boolean pivotEncoderValid = false;

  // path follower
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer(RobotState state) {
    this.state = state;

    // bind internal drivetrain buffer of historical poses
    state.setHistoricalPoseSupplier(drivetrain::getPoseAt);

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

    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (Constants.Settings.testingMode) {
      configureTesting();
    } else {
      configureBindings();
      configureTriggers();
    }

    if (!Constants.Settings.drivingDisabled) {
      configureDriveBindings();
    }
  }

  /** Use this method to configure bindings and triggers for testing mode. */
  private void configureTesting() {
    // reset the intent
    controllerDriver.x().onTrue(state.setIntent(Intent.NONE));

    controllerDriver.rightBumper().onTrue(state.setIntent(Intent.CORAL_EJECT));
    controllerDriver.povUp().onTrue(state.setIntent(Intent.CORAL_SCORE_AUTO));
    controllerDriver.povDown().onTrue(state.setIntent(Intent.CORAL_SCORE_L1));
    controllerDriver.b().onTrue(state.setIntent(Intent.CORAL_FLOOR_INTAKE));

    // controllerDriver.povLeft().onTrue(state.setIntent(Intent.ALGAE_INTAKE));
    // controllerDriver.povRight().onTrue(state.setIntent(Intent.ALGAE_EJECT));

    // controllerDriver.povUp().onTrue(new InstantCommand(() -> pivot.setAngle(90), pivot));
    // controllerDriver.povDown().onTrue(new InstantCommand(() -> pivot.setAngle(10), pivot));

    // controllerDriver.povUp().onTrue(new InstantCommand(() -> elevator.setPosition(12),
    // elevator));
    // controllerDriver.povDown().onTrue(new InstantCommand(() -> elevator.setPosition(0),
    // elevator));

    // controllerDriver.povLeft().onTrue(new InstantCommand(() -> pivot.setAngle(0), pivot));
    // controllerDriver.povRight().onTrue(new InstantCommand(() -> pivot.setAngle(80), pivot));

    // controllerDriver.y().onTrue(state.setIntent(Intent.CORAL_FLOOR_INTAKE));
    // controllerDriver.povUp().onTrue(state.setIntent(Intent.CORAL_SCORE_L1));

    // controllerDriver.a().onTrue(state.setIntent(Intent.CORAL_FLOOR_INTAKE));
    // controllerDriver.b().onTrue(new InstantCommand(elevator::reset));

    // state.shouldScoreCoralL1.whileTrue(superstructure.setPosition(Position.L1));
    // state.shouldIntakeCoralFloor.whileTrue(superstructure.setPosition(Position.STOWED));

    // controllerDriver.rightBumper().onTrue(state.setIntent(Intent.CORAL_EJECT));

    // intake algae
    state.shouldIntakeAlgae.whileTrue(manipulator.intakeAlgae());

    // eject algae
    state.shouldEjectAlgae.whileTrue(manipulator.ejectAlgae());

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

    // reset intent once coral has been acquired
    state.shouldIntakeCoral.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.NONE));

    state.shouldEjectCoral.and(superstructure.hasCoral).whileTrue(manipulator.ejectCoral());

    state
        .shouldEjectCoral
        .and(superstructure.hasCoral.negate())
        .onTrue(state.setIntent(Intent.NONE));

    // brake to prevent robot backing when operating elevator/pivot
    state.shouldScoreCoral.and(state.shouldScoreCoralL1.negate()).whileTrue(drivetrain.brake());

    // score coral at L1
    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L1));

    state
        .shouldScoreCoralAuto
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setAutoScoringPosition());

    // state
    //     .shouldScoreCoralAuto
    //     .and(superstructure.hasCoral)
    //     .and(superstructure.targetsL4.negate())
    //     .and(superstructure.canScoreAuto)
    //     .whileTrue(manipulator.ejectCoral());

    // state
    //     .shouldScoreCoralL1
    //     .and(superstructure.hasCoral)
    //     .and(superstructure.canScoreL1)
    //     .whileTrue(manipulator.ejectCoral());

    // [default behavior] stow when no intent is specified
    state.idle.whileTrue(superstructure.setPosition(Position.STOWED));
  }

  /** Use this method to configure controller bindings. */
  private void configureBindings() {}

  /** Use this method to configure controller bindings related to driving. */
  private void configureDriveBindings() {
    // note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    // drive forward with negative Y (forward)
                    .withVelocityX(-controllerDriver.getLeftY() * maxSpeedMetersPerSecond)
                    // drive left with negative X (left)
                    .withVelocityY(-controllerDriver.getLeftX() * maxSpeedMetersPerSecond)
                    // drive counterclockwise with negative X (left)
                    .withRotationalRate(
                        controllerDriver.getRightX() * maxAngularRateRadiansPerSecond)));

    // reset the field-centric heading on left bumper press
    controllerDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // set brake position (X) when holding A
    controllerDriver.a().whileTrue(drivetrain.brake());

    // controllerDriver
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(
    //                         -controllerDriver.getLeftX(), -controllerDriver.getLeftY()))));

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      controllerDriver
          .povUp()
          .whileTrue(
              drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

      controllerDriver
          .povDown()
          .whileTrue(
              drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

      // controllerDriver.leftBumper().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
      // controllerDriver.rightBumper().whileTrue(new WheelDiameterCharacterization(drivetrain,
      // -1));

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
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // intent-driven triggers

    // eject coral
    state
        .shouldEjectCoral
        .and(superstructure.hasCoral)
        .and(superstructure.canEject)
        .whileTrue(manipulator.ejectCoral());

    // reset intent once coral has been ejected
    state
        .shouldEjectCoral
        .and(superstructure.hasCoral.negate())
        .onTrue(state.setIntent(Intent.NONE));

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

    // score coral at L1
    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.L1));

    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL1)
        .whileTrue(manipulator.ejectCoral());

    // score coral via auto target
    state
        .shouldScoreCoralAuto
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setAutoScoringPosition());

    state
        .shouldScoreCoralAuto
        .and(superstructure.hasCoral)
        .and(superstructure.targetsL4.negate())
        .and(superstructure.canScoreAuto)
        .whileTrue(manipulator.ejectCoral());

    state
        .shouldScoreCoralAuto
        .and(superstructure.hasCoral)
        .and(superstructure.targetsL4)
        .and(superstructure.canScoreL4)
        .onTrue(superstructure.setPosition(Position.L4_SCORED));

    // reset intent once coral has been scored
    state
        .shouldScoreCoral
        .and(superstructure.hasCoral.negate())
        .onTrue(state.setIntent(Intent.NONE));

    // [default behavior] stow when no intent is specified
    state.idle.whileTrue(superstructure.setPosition(Position.STOWED));

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
  // spotless:on
}
