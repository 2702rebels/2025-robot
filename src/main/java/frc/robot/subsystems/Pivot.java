package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Pivot extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 1;

  // positional tolerance
  private static final double positionalTolerance = 0.5;

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

  /**
   * Gets the algae presence sensor.
   *
   * <p>Limit switch is connected to the data port on the SparkMax controller.
   */
  public SparkLimitSwitch getAlgaePresenceSensor() {
    return motorLeader.getForwardLimitSwitch();
  }

  /** Trigger for pivot is stowed position. */
  @Logged(name = "IsStowed", importance = Importance.INFO)
  public final Trigger isStowed = new Trigger(() -> isAtAngle(0)); // TODO: value

  /** Trigger for pivot position safe for ejection. */
  @Logged(name = "CanEject", importance = Importance.INFO)
  public final Trigger canEject = new Trigger(() -> isAtAngle(0)); // TODO: value

  public void setAngle(double value) {
    controller.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean isAtAngle(double value) {
    return MathUtil.isNear(value, getAngle(), positionalTolerance);
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
