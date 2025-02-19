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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
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
          .withDeadband(maxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(maxAngularRateRadiansPerSecond * 0.1) // add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final RobotState state;

  private final Telemetry logger = new Telemetry();

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
          Constants.CanID.MOTOR_MANIPULATOR_VERTICAL, Constants.CanID.MOTOR_MANIPULATOR_HORIZONTAL);

  @Logged(name = "Superstructure", importance = Importance.INFO)
  private final Superstructure superstructure = new Superstructure(elevator, pivot);

  @Logged(name = "ObjectDetector", importance = Importance.INFO)
  private final ObjectDetector objectDetector = new ObjectDetector();

  // path follower
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer(RobotState state) {
    this.state = state;

    // configure DogLog logging
    DogLog.setOptions(new DogLogOptions().withCaptureConsole(true).withLogExtras(true));

    // log metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    configureTriggers();
  }

  /** Use this method to configure controller bindings. */
  private void configureBindings() {
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
                        -controllerDriver.getRightX() * maxAngularRateRadiansPerSecond)));

    controllerDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    controllerDriver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -controllerDriver.getLeftY(), -controllerDriver.getLeftX()))));

    controllerDriver
        .pov(0)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    controllerDriver
        .pov(180)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
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

    // reset the field-centric heading on left bumper press
    controllerDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::update);
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // intent-driven
    state
        .shouldEjectCoral
        .and(manipulator.hasCoral)
        .and(pivot.canEject)
        .whileTrue(manipulator.ejectCoral());

    state
        .shouldIntakeCoralFloor
        .and(manipulator.hasCoral.negate())
        .and(superstructure.canFloorIntake)
        .whileTrue(manipulator.intakeCoral());

    state
        .shouldIntakeCoralStation
        .and(manipulator.hasCoral.negate())
        .and(superstructure.canStationIntake)
        .whileTrue(manipulator.intakeCoral());

    state
        .shouldIntakeCoralFloor
        .and(manipulator.hasCoral.negate())
        .onTrue(superstructure.setPosition(Position.FLOOR));

    state
        .shouldIntakeCoralStation
        .and(manipulator.hasCoral.negate())
        .onTrue(superstructure.setPosition(Position.STATION));

    state
        .shouldIntakeCoralFloor
        .or(state.shouldIntakeCoralStation)
        .and(manipulator.hasCoral)
        .onTrue(state.setIntent(Intent.NONE));

    state
        .shouldScoreCoral
        .and(manipulator.hasCoral)
        .onTrue(superstructure.setAutoScoringPosition());

    state
        .shouldScoreCoral
        .and(manipulator.hasCoral)
        .and(superstructure.canScoreL2)
        .whileTrue(manipulator.ejectCoral());

    // [default behavior] stow the game piece when no intent is specified
    state.idle.and(manipulator.hasCoral).whileTrue(superstructure.setPosition(Position.STOWED));

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
