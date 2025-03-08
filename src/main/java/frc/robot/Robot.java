// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Tunable;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  @Logged(name = "Container")
  private final RobotContainer robotContainer;

  @Logged(name = "State")
  private final RobotState robotState;

  private final Vision vision;

  private final double kVisionCutoff = 60 * 1000; // (60ms) in microseconds
  private final double kVisionLinearTolerance = Units.feetToMeters(4);
  private final double kVisionRotationTolerance = Units.degreesToRadians(90);

  private final boolean kUseVision = true;

  // accept first vision update as ground truth
  private boolean visionSynced = false;

  public Robot() {
    robotState = RobotState.getInstance();
    vision = Vision.getInstance();
    robotContainer = new RobotContainer(robotState);

    robotContainer.drivetrain.setVisionMeasurementStdDevs(
        VecBuilder.fill(
            Units.inchesToMeters(4), Units.inchesToMeters(4), Units.degreesToRadians(60)));

    DataLogManager.start();

    // https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html
    Epilogue.configure(
        config -> {
          config.root = "Robot";

          if (isSimulation()) {
            // if running in simulation, then we'd want to re-throw any errors that
            // occur so we can debug and fix them!
            config.errorHandler = ErrorHandler.crashOnError();
          }

          // change to CRITICAL for more detailed logging (where supported)
          config.minimumImportance = Logged.Importance.DEBUG;
        });

    Epilogue.bind(this);
  }

  @Override
  public void robotInit() {
    robotContainer.initValidation();
  }

  @Override
  public void robotPeriodic() {
    Tunable.periodic();

    robotState.enterPeriodic();
    CommandScheduler.getInstance().run();
    robotState.exitPeriodic();

    // update odometry tracker
    robotState.updateOdometry(
        RobotController.getFPGATime(),
        robotContainer.drivetrain.getPose(),
        robotContainer.drivetrain.getFieldSpeeds());

    if (kUseVision) {
      visionPeriodic();
    }
  }

  private void visionPeriodic() {
    final var visionState = robotState.getVisionState();
    if (visionState.isEmpty()) {
      return;
    }

    final var visionPoseValue = visionState.get().pose;
    final var visionPoseTimestamp = visionPoseValue.timestamp();
    final var visionPose = visionPoseValue.value();

    // invalid pose
    if (visionPoseTimestamp <= 0) {
      return;
    }

    // pose too old
    if (RobotController.getFPGATime() - visionPoseTimestamp > kVisionCutoff) {
      return;
    }

    final var driveState = robotContainer.drivetrain.getState();
    final double omegaRotationsPerSecond =
        Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    // rotating too fast
    if (omegaRotationsPerSecond > 1) {
      return;
    }

    if (visionSynced) {
      // no historical pose
      final var robotPoseAtTimestamp = robotState.getPoseAt(visionPoseTimestamp);
      if (robotPoseAtTimestamp.isEmpty()) {
        return;
      }

      final var twist = visionPose.log(robotPoseAtTimestamp.get());
      if (Math.abs(twist.dx) > kVisionLinearTolerance
          || Math.abs(twist.dy) > kVisionLinearTolerance
          || Math.abs(twist.dtheta) > kVisionRotationTolerance) {
        return;
      }
    } else {
      visionSynced = true;
      robotContainer.drivetrain.resetPose(visionPose);
    }

    robotContainer.drivetrain.addVisionMeasurement(
        visionPose, Utils.fpgaToCurrentTime(visionPoseTimestamp / 1000000.0));
  }

  @Override
  public void disabledInit() {
    vision.disableLogging();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    vision.enableLogging();
    RobotState.getInstance().enterAutonomous();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
