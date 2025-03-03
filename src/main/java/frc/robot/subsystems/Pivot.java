package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.RevUtil;
import frc.robot.lib.Tunable;

public class Pivot extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 0.75;
  private static final double kD = 0.05;
  private static final double kRampRate = 0.25;

  // positional tolerance
  private static final double positionalTolerance = 2.5; // degrees

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedSparkMax motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedSparkMax motorFollower;

  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;

  public Pivot(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(rampRate.get());

    final var signalsConfig = new SignalsConfig().absoluteEncoderPositionAlwaysOn(true);

    motorLeader =
        new LoggedSparkMax(
            leaderCanID,
            (config) -> {
              config.inverted(true);

              config
                  .absoluteEncoder
                  .zeroCentered(true)
                  .inverted(true)
                  .positionConversionFactor(360) // degrees
                  .velocityConversionFactor(360 / 60); // degrees/second

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .velocityFF(0)
                  .p(kP)
                  .d(kD);

              config.apply(commonConfig);
              config.signals.apply(signalsConfig);
            });

    motorFollower =
        new LoggedSparkMax(
            followerCanID,
            (config) -> {
              // follow leader's output (inverted)
              config.apply(commonConfig).follow(motorLeader, true);
              config.signals.apply(signalsConfig);
            });

    encoder = motorLeader.getAbsoluteEncoder();
    controller = motorLeader.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Pivot", motorLeader, ClosedLoopSlot.kSlot0);
  }

  private void updateRampRate(double value) {
    final var newConfig = new SparkMaxConfig();
    newConfig.closedLoopRampRate(value);
    RevUtil.configEphemeral(motorLeader, newConfig);
    RevUtil.configEphemeral(motorFollower, newConfig);
  }

  /**
   * Gets the algae presence sensor.
   *
   * <p>Limit switch is connected to the data port on the SparkMax controller.
   */
  public SparkLimitSwitch getAlgaePresenceSensor() {
    return motorLeader.getForwardLimitSwitch();
  }

  public void setAngle(double valueDegrees) {
    controller.setReference(valueDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean isAtAngle(double valueDegrees) {
    return isAtAngle(valueDegrees, positionalTolerance);
  }

  public boolean isAtAngle(double valueDegrees, double toleranceDegrees) {
    return MathUtil.isNear(valueDegrees, getAngle(), toleranceDegrees);
  }

  @Logged(name = "Angle", importance = Importance.INFO)
  public double getAngle() {
    return encoder.getPosition();
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return encoder.getVelocity();
  }

  // spotless:off
  private Tunable.DoubleValue rampRate = Tunable.value("Pivot/RampRate", kRampRate, this::updateRampRate);
  // spotless:on
}
