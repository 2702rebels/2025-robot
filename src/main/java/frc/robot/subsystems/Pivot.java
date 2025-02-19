package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Pivot extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 1;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedSparkMax motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedSparkMax motorFollower;

  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;

  public Pivot(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(60)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .signals
            .absoluteEncoderPositionAlwaysOn(true);

    motorLeader =
        new LoggedSparkMax(
            leaderCanID,
            (config) -> {
              config
                  .absoluteEncoder
                  .positionConversionFactor(2 * Math.PI) // radians
                  .velocityConversionFactor(2 * Math.PI / 60); // radians/second

              config
                  .apply(commonConfig)
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .velocityFF(0)
                  .p(kP);
            });

    motorFollower =
        new LoggedSparkMax(
            followerCanID,
            (config) -> {
              // follow leader's output (inverted)
              config.apply(commonConfig).follow(motorLeader, true);
            });

    encoder = motorLeader.getAbsoluteEncoder();
    controller = motorLeader.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Pivot", motorLeader, ClosedLoopSlot.kSlot0);
  }

  /** Trigger for pivot is stowed position. */
  @Logged(name = "IsStowed", importance = Importance.INFO)
  public final Trigger isStowed = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position safe for ejection. */
  @Logged(name = "CanEject", importance = Importance.INFO)
  public final Trigger canEject = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for floor intake */
  @Logged(name = "CanFloorIntake", importance = Importance.INFO)
  public final Trigger canFloorIntake = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for station intake */
  @Logged(name = "CanStationIntake", importance = Importance.INFO)
  public final Trigger canStationIntake = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for scoring at L1 */
  @Logged(name = "CanScoreL1", importance = Importance.INFO)
  public final Trigger canScoreL1 = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for scoring at L2 */
  @Logged(name = "CanScoreL2", importance = Importance.INFO)
  public final Trigger canScoreL2 = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for scoring at L3 */
  @Logged(name = "CanScoreL3", importance = Importance.INFO)
  public final Trigger canScoreL3 = new Trigger(() -> getAngle() > 0); // TODO: value

  /** Trigger for pivot position ready for scoring at L4 */
  @Logged(name = "CanScoreL4", importance = Importance.INFO)
  public final Trigger canScoreL4 = new Trigger(() -> getAngle() > 0); // TODO: value

  public void setAngle(double value) {
    controller.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public double getAngle() {
    return encoder.getPosition();
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
