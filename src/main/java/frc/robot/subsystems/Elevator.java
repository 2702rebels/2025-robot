package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
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

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 0.5;
  private static final double kD = 0;
  private static final double kMaxVelocity = 64; // inch/sec
  private static final double kMaxAcceleration = 256; // inch/sec.sq
  private static final double kRampRate = 0.25;

  // conversion factor between rotation and linear movement
  // 3" wheel diameter, 8:1 reduction ratio
  private static final double conversionFactor = Math.PI * 3 / 8;

  // positional tolerance
  private static final double positionalTolerance = 0.5;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedSparkMax motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedSparkMax motorFollower;

  private final RelativeEncoder encoderLeader;
  private final RelativeEncoder encoderFollower;
  private final SparkClosedLoopController controller;

  public Elevator(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(rampRate.get());

    final var signalsConfig = new SignalsConfig().primaryEncoderPositionAlwaysOn(true);

    final var encoderConfig =
        new EncoderConfig()
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60);

    motorLeader =
        new LoggedSparkMax(
            leaderCanID,
            (config) -> {
              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(kP)
                  .d(kD)
                  .maxMotion
                  .maxVelocity(kMaxVelocity)
                  .maxAcceleration(kMaxAcceleration)
                  .allowedClosedLoopError(positionalTolerance);

              config.apply(commonConfig);
              config.encoder.apply(encoderConfig);
              config.signals.apply(signalsConfig);
            });

    motorFollower =
        new LoggedSparkMax(
            followerCanID,
            (config) -> {
              // follow leader's output (non-inverted)
              config.apply(commonConfig).follow(motorLeader);
              config.encoder.apply(encoderConfig);
              config.signals.apply(signalsConfig);
            });

    encoderLeader = motorLeader.getEncoder();
    encoderFollower = motorFollower.getEncoder();
    controller = motorLeader.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Elevator", motorLeader, ClosedLoopSlot.kSlot0);

    reset();
  }

  private void updateRampRate(double value) {
    final var newConfig = new SparkMaxConfig();
    newConfig.closedLoopRampRate(value);
    RevUtil.configEphemeral(motorLeader, newConfig);
    RevUtil.configEphemeral(motorFollower, newConfig);
  }

  public void reset() {
    encoderLeader.setPosition(0);
  }

  public void setPosition(double value) {
    // use ControlType.kMAXMotionPositionControl for MAX Motion
    // however, it does not seem to work well at all
    controller.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean isAtPosition(double value) {
    return isAtPosition(value, positionalTolerance);
  }

  public boolean isAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getPosition(), tolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return getLeaderPosition();
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return getLeaderVelocity();
  }

  @Logged(name = "Leader/Position", importance = Importance.INFO)
  public double getLeaderPosition() {
    return encoderLeader.getPosition();
  }

  @Logged(name = "Leader/Velocity", importance = Importance.INFO)
  public double getLeaderVelocity() {
    return encoderLeader.getVelocity();
  }

  @Logged(name = "Follower/Position", importance = Importance.INFO)
  public double getFollowerPosition() {
    return encoderFollower.getPosition();
  }

  @Logged(name = "Follower/Velocity", importance = Importance.INFO)
  public double getFollowerVelocity() {
    return encoderFollower.getVelocity();
  }

  // spotless:off
  private Tunable.DoubleValue rampRate = Tunable.value("Elevator/RampRate", kRampRate, this::updateRampRate);
  // spotless:on
}
