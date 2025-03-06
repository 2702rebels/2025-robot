// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.util.Units;
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

  private final boolean kUseVision = false;

  public Robot() {
    robotState = RobotState.getInstance();
    vision = Vision.getInstance();
    robotContainer = new RobotContainer(robotState);

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
      final var visionState = robotState.getVisionState();
      if (visionState.isPresent()) {
        var driveState = robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRotationsPerSecond =
            Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        final var visionPose = visionState.get().pose;
        final var visionPoseTimestamp = visionPose.timestamp();

        if (visionPoseTimestamp > 0 && omegaRotationsPerSecond < 1) {
          robotContainer.drivetrain.addVisionMeasurement(
              visionPose.value(), Utils.fpgaToCurrentTime(visionPoseTimestamp / 1000000.0));
        }
      }
    }
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
