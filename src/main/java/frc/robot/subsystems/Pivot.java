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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Pivot extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 0.01;
  private static final double kD = 0.;

  private static final double kS = 0;
  private static final double kG = 0;
  private static final double kV = 0;
  private static final double kA = 0;
  private static final double kMaxVelocity = 180; // degrees/s
  private static final double kMaxAcceleration = kMaxVelocity * 4; // degrees/s^2

  // maximum safe angle (degrees)
  private static final double maxAngle = 115;

  // tolerance (degrees)
  private static final double tolerance = 2.5;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedSparkMax motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedSparkMax motorFollower;

  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;

  private final ArmFeedforward ffController;
  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  public Pivot(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake);

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

    ffController = new ArmFeedforward(kS, kG, kV, kA);
    trapezoidProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // make FF configuration tunable
    Tunable.bind("Pivot/FF", ffController);
  }

  private void updateMaxVelocity(double value) {
    trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(value, maxAcceleration.get()));
  }

  private void updateMaxAcceleration(double value) {
    trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), value));
  }

  @Override
  public void periodic() {
    goal.position = setpoint;
    state.position = getAngle();
    state.velocity = getVelocity();

    // this calculate method is doing at least two new allocations... :(
    state = trapezoidProfile.calculate(Constants.cyclePeriodSeconds, state, goal);
    final var ff = ffController.calculate(state.position, state.velocity);

    controller.setReference(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
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
    setpoint = Math.min(valueDegrees, maxAngle);
  }

  public boolean isAtAngle(double valueDegrees) {
    return isAtAngle(valueDegrees, tolerance);
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

  @Logged(name = "State/Angle", importance = Importance.DEBUG)
  public double stateAngle() {
    return state.position;
  }

  @Logged(name = "State/Velocity", importance = Importance.DEBUG)
  public double stateVelocity() {
    return state.velocity;
  }

  // spotless:off
  private Tunable.DoubleValue maxVelocity = Tunable.value("Pivot/FF/MaxVelocity", kMaxVelocity, this::updateMaxVelocity);
  private Tunable.DoubleValue maxAcceleration = Tunable.value("Pivot/FF/MaxAcceleration", kMaxAcceleration, this::updateMaxAcceleration);
  // spotless:on
}
