package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

/**
 * Controls motors responsible for manipulating game pieces (coral, algae), enables intake/eject
 * motion of the game piece.
 */
public final class Manipulator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double __kCoralIntakeVelocity = 0.75;
  private static final double __kCoralIntakeVelocityGuide = 0.75;
  private static final double __kCoralEjectVelocity = 0.4;

  private static final double __kAlgaeIntakeVelocity = 0.5;
  private static final double __kAlgaeEjectVelocity = 0.5;
  private static final double __kAlgaeHoldVelocity = 0.1;

  private static final double kGuideGearRatio = 7;
  private static final double kCoralGearRatio = 7;

  @Logged(name = "Guide", importance = Importance.INFO)
  private final LoggedSparkMax motorGuide;

  @Logged(name = "Coral", importance = Importance.INFO)
  private final LoggedSparkMax motorCoral;

  public Manipulator(int guideCanID, int coralCanID) {
    motorGuide =
        new LoggedSparkMax(
            guideCanID,
            (config) -> {
              config
                  .smartCurrentLimit(40)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kCoast)
                  .signals
                  .primaryEncoderVelocityAlwaysOn(true);

              // conversion factor based on the gear ratio
              config
                  .encoder
                  .positionConversionFactor(1 / kGuideGearRatio)
                  .velocityConversionFactor(1 / kGuideGearRatio);
            });

    motorCoral =
        new LoggedSparkMax(
            coralCanID,
            (config) -> {
              config
                  .smartCurrentLimit(40)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kCoast)
                  .signals
                  .primaryEncoderVelocityAlwaysOn(true);

              // conversion factor based on the gear ratio
              config
                  .encoder
                  .positionConversionFactor(1 / kCoralGearRatio)
                  .velocityConversionFactor(1 / kCoralGearRatio);
            });
  }

  private void stopMotors() {
    motorCoral.stopMotor();
    motorGuide.stopMotor();
  }

  /**
   * Gets the coral presence sensor.
   *
   * <p>Limit switch is connected to the data port on the SparkMax controller.
   */
  public SparkLimitSwitch getCoralPresenceSensor() {
    return motorGuide.getForwardLimitSwitch();
  }

  public Command intakeCoral() {
    return runEnd(
            () -> {
              motorCoral.set(-coralIntakeVelocity.get());
              motorGuide.set(-coralIntakeVelocityGuide.get());
            },
            this::stopMotors)
        .withName("IntakeCoral");
  }

  public Command ejectCoral() {
    return runEnd(
            () -> {
              motorCoral.set(coralEjectVelocity.get());
              motorGuide.stopMotor();
            },
            this::stopMotors)
        .withName("EjectCoral");
  }

  public Command intakeAlgae() {
    return runEnd(
            () -> {
              motorCoral.stopMotor();
              motorGuide.set(algaeIntakeVelocity.get());
            },
            this::stopMotors)
        .withName("IntakeAlgae");
  }

  public Command ejectAlgae() {
    return runEnd(
            () -> {
              motorCoral.stopMotor();
              motorGuide.set(-algaeEjectVelocity.get());
            },
            this::stopMotors)
        .withName("EjectAlgae");
  }

  public Command holdAlgae() {
    return runEnd(
            () -> {
              motorCoral.stopMotor();
              motorGuide.set(algaeHoldVelocity.get());
            },
            this::stopMotors)
        .withName("HoldAlgae");
  }

  public Command stop() {
    return run(this::stopMotors).withName("ManipulatorStop");
  }

  // spotless:off
  private static Tunable.DoubleValue coralIntakeVelocity = Tunable.value("Manipulator/Coral/IntakeVelocity", __kCoralIntakeVelocity);
  private static Tunable.DoubleValue coralIntakeVelocityGuide = Tunable.value("Manipulator/Coral/IntakeVelocityGuide", __kCoralIntakeVelocityGuide);
  private static Tunable.DoubleValue coralEjectVelocity = Tunable.value("Manipulator/Coral/EjectVelocity", __kCoralEjectVelocity);

  private static Tunable.DoubleValue algaeIntakeVelocity = Tunable.value("Manipulator/Algae/IntakeVelocity", __kAlgaeIntakeVelocity);
  private static Tunable.DoubleValue algaeEjectVelocity = Tunable.value("Manipulator/Algae/EjectVelocity", __kAlgaeEjectVelocity);
  private static Tunable.DoubleValue algaeHoldVelocity = Tunable.value("Manipulator/Algae/HoldVelocity", __kAlgaeHoldVelocity);
  // spotless:on
}
