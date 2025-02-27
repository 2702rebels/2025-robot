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
  private static final double CORAL_INTAKE_VELOCITY = 0.65; // ~1000 RPM
  private static final double CORAL_INTAKE_VELOCITY_GUIDE = 0.75; // ~1000 RPM
  private static final double CORAL_EJECT_VELOCITY = 0.5;

  private static final double ALGAE_INTAKE_VELOCITY = 0.5;
  private static final double ALGAE_EJECT_VELOCITY = 0.5;

  private static final double kGuideGearRatio = 7;
  private static final double kCoralGearRatio = 7;

  @Logged(name = "Guide", importance = Importance.INFO)
  private final LoggedSparkMax motorGuide;

  @Logged(name = "Coral", importance = Importance.INFO)
  private final LoggedSparkMax motorCoral;

  public Manipulator(int guideCanID, int coralCanID) {
    motorGuide =
        new LoggedSparkMax(
            coralCanID,
            (config) -> {
              config
                  .smartCurrentLimit(30)
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
                  .smartCurrentLimit(50)
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
              motorGuide.set(-algaeIntakeVelocity.get());
            },
            this::stopMotors)
        .withName("IntakeAlgae");
  }

  public Command ejectAlgae() {
    return runEnd(
            () -> {
              motorCoral.stopMotor();
              motorCoral.set(algaeEjectVelocity.get());
            },
            this::stopMotors)
        .withName("EjectAlgae");
  }

  public Command stop() {
    return run(this::stopMotors).withName("ManipulatorStop");
  }

  // spotless:off
  private static Tunable.DoubleValue coralIntakeVelocity = Tunable.value("Manipulator/Coral/IntakeVelocity", CORAL_INTAKE_VELOCITY);
  private static Tunable.DoubleValue coralIntakeVelocityGuide = Tunable.value("Manipulator/Coral/IntakeVelocityGuide", CORAL_INTAKE_VELOCITY_GUIDE);
  private static Tunable.DoubleValue coralEjectVelocity = Tunable.value("Manipulator/Coral/EjectVelocity", CORAL_EJECT_VELOCITY);

  private static Tunable.DoubleValue algaeIntakeVelocity = Tunable.value("Manipulator/Algae/IntakeVelocity", ALGAE_INTAKE_VELOCITY);
  private static Tunable.DoubleValue algaeEjectVelocity = Tunable.value("Manipulator/Algae/EjectVelocity", ALGAE_EJECT_VELOCITY);
  // spotless:on
}
