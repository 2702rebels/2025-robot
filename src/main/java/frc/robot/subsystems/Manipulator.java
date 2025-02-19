package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RevConstants;
import frc.robot.lib.RevVelocityController;
import frc.robot.lib.Tunable;

/**
 * Controls motors responsible for manipulating game pieces (coral, algae), enables intake/eject
 * motion of the game piece.
 */
public final class Manipulator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double VERTICAL_kP = 1;
  private static final double HORIZONTAL_kP = 1;
  private static final double CORAL_INTAKE_VELOCITY = 1000;
  private static final double CORAL_EJECT_VELOCITY = 1000;
  private static final double ALGAE_INTAKE_VELOCITY = 1000;
  private static final double ALGAE_EJECT_VELOCITY = 1000;

  private static final double kVerticalGearRatio = 7;
  private static final double kHorizontalGearRatio = 7;

  // delay on the game piece presence trigger to allow it fully
  // leave the mechanism before shutting down motors, etc.
  private static final double kHasGamePieceDelaySeconds = 0.2;

  @Logged(name = "Vertical", importance = Importance.INFO)
  private final RevVelocityController vertical;

  @Logged(name = "Horizontal", importance = Importance.INFO)
  private final RevVelocityController horizontal;

  private final SparkLimitSwitch coralSwitch;
  private final SparkLimitSwitch algaeSwitch;

  public Manipulator(int verticalMotorCanID, int horizontalMotorCanID) {
    vertical =
        new RevVelocityController(
            verticalMotorCanID,
            "Manipulator/Vertical",
            kVerticalGearRatio,
            VERTICAL_kP,
            0,
            0,
            1 / RevConstants.NEO550.kV);
    horizontal =
        new RevVelocityController(
            horizontalMotorCanID,
            "Manipulator/Horizontal",
            kHorizontalGearRatio,
            HORIZONTAL_kP,
            0,
            0,
            1 / RevConstants.NEO550.kV);

    coralSwitch = horizontal.getForwardLimitSwitch();
    algaeSwitch = vertical.getForwardLimitSwitch();

    // spotless:off
    hasCoral = new Trigger(coralSwitch::isPressed).debounce(kHasGamePieceDelaySeconds, DebounceType.kFalling);
    hasAlgae = new Trigger(algaeSwitch::isPressed).debounce(kHasGamePieceDelaySeconds, DebounceType.kFalling);
    // spotless:on
  }

  private void stopMotors() {
    horizontal.stop();
    vertical.stop();
  }

  /** Trigger for coral presence. */
  @Logged(name = "HasCoral", importance = Importance.INFO)
  public final Trigger hasCoral;

  /** Trigger for algae presence. */
  @Logged(name = "HasAlgae", importance = Importance.INFO)
  public final Trigger hasAlgae;

  public Command intakeCoral() {
    return runEnd(
            () -> {
              horizontal.setVelocity(coralIntakeVelocity.get());
              vertical.setVelocity(coralIntakeVelocity.get());
            },
            this::stopMotors)
        .withName("IntakeCoral");
  }

  public Command ejectCoral() {
    return runEnd(
            () -> {
              horizontal.setVelocity(-coralEjectVelocity.get());
              vertical.setVelocity(-coralEjectVelocity.get());
            },
            this::stopMotors)
        .withName("EjectCoral");
  }

  public Command intakeAlgae() {
    return runEnd(
            () -> {
              horizontal.setVelocity(algaeIntakeVelocity.get());
              vertical.stop();
            },
            this::stopMotors)
        .withName("IntakeAlgae");
  }

  public Command ejectAlgae() {
    return runEnd(
            () -> {
              horizontal.setVelocity(-algaeEjectVelocity.get());
              vertical.stop();
            },
            this::stopMotors)
        .withName("EjectAlgae");
  }

  public Command stop() {
    return run(this::stopMotors).withName("ManipulatorStop");
  }

  // spotless:off
  private static Tunable.DoubleValue coralIntakeVelocity = Tunable.value("Manipulator/Coral/IntakeVelocity", CORAL_INTAKE_VELOCITY);
  private static Tunable.DoubleValue coralEjectVelocity = Tunable.value("Manipulator/Coral/EjectVelocity", CORAL_EJECT_VELOCITY);
  private static Tunable.DoubleValue algaeIntakeVelocity = Tunable.value("Manipulator/Algae/IntakeVelocity", ALGAE_INTAKE_VELOCITY);
  private static Tunable.DoubleValue algaeEjectVelocity = Tunable.value("Manipulator/Algae/EjectVelocity", ALGAE_EJECT_VELOCITY);
  // spotless:on
}
