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

  private final boolean kUseLimelight = false;

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

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var driveState = robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        robotContainer.drivetrain.addVisionMeasurement(
            llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
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
