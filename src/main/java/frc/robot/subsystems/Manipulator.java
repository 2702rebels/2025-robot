package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Tunable;

/**
 * Controls motors responsible for manipulating game pieces (coral, algae), enables intake/eject
 * motion of the game piece.
 */
public final class Manipulator extends SubsystemBase {
  // spotless:off
  private static Tunable.DoubleValue intakeVelocity = Tunable.value("Manipulator/IntakeVelocity", 0);
  private static Tunable.DoubleValue ejectVelocity = Tunable.value("Manipulator/EjectVelocity", 0);
  // spotless:on

  public Command startIntaking() {
    return runOnce(
        () -> {
          // set motor PID controller
        });
  }

  public Command startEjecting() {
    return runOnce(
        () -> {
          // set motor PID controller
        });
  }

  public Command stop() {
    return run(
        () -> {
          // stop motor
        });
  }
}
