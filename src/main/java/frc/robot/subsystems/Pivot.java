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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Pivot extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 0.007;
  private static final double __kD = 0.;

  private static final double __kS = 0;
  private static final double __kG = 0.7;
  private static final double __kV = 0.012;
  private static final double __kA = 0;
  private static final double __kMaxVelocity = 480; // degrees/s
  private static final double __kMaxAcceleration = 1920; // degrees/s^2

  // maximum safe angle (degrees)
  private static final double maxAngle = 140;

  // tolerance (degrees)
  private static final double tolerance = 3;

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
            .smartCurrentLimit(50)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake);

    final var signalsConfig = new SignalsConfig().absoluteEncoderPositionAlwaysOn(true);

    motorLeader =
        new LoggedSparkMax(
            leaderCanID,
            (config) -> {
              config.inverted(true);

              // IMPORTANT!
              // REV documentation claims that velocity should be reported in RPMs,
              // however, we observe values reported in what seems to be rotations/second
              // velocityConversionFactor is therefore set to 360 not 60
              config
                  .absoluteEncoder
                  .zeroCentered(true)
                  .inverted(true)
                  .positionConversionFactor(360) // degrees
                  .velocityConversionFactor(360); // degrees/second

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .velocityFF(0)
                  .p(__kP)
                  .d(__kD);

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

    ffController = new ArmFeedforward(__kS, __kG, __kV, __kA);
    trapezoidProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(kMaxVelocity.get(), kMaxAcceleration.get()));

    // make FF configuration tunable
    Tunable.bind("Pivot/FF", ffController);
  }

  private void updateMaxVelocity(double value) {
    trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(value, kMaxAcceleration.get()));
  }

  private void updateMaxAcceleration(double value) {
    trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVelocity.get(), value));
  }

  @Override
  public void periodic() {
    goal.position = setpoint;
    state.position = getAngle();
    state.velocity = getVelocity();

    // this calculate method is doing at least two new allocations... :(
    state = trapezoidProfile.calculate(Constants.cyclePeriodSeconds, state, goal);

    // do not include velocity component when at setpoint (within tolerance)
    final var ff =
        ffController.calculate(
            Units.degreesToRadians(state.position), isAtAngle(setpoint) ? 0 : state.velocity);

    controller.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
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
  private Tunable.DoubleValue kMaxVelocity = Tunable.value("Pivot/FF/MaxVelocity", __kMaxVelocity, this::updateMaxVelocity);
  private Tunable.DoubleValue kMaxAcceleration = Tunable.value("Pivot/FF/MaxAcceleration", __kMaxAcceleration, this::updateMaxAcceleration);
  // spotless:on
}
