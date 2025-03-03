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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 0.01;
  private static final double kD = 0;

  private static final double kS = 0;
  private static final double kG = 0;
  private static final double kV = 0;
  private static final double kA = 0;
  private static final double kMaxVelocity = 64; // inch/sec
  private static final double kMaxAcceleration = 256; // inch/sec.sq

  // conversion factor between rotation and linear movement
  // 3" wheel diameter, 8:1 reduction ratio
  private static final double conversionFactor = Math.PI * 3 / 8;

  // maximum safe travel
  private static final double maxTravel = 70;

  // tolerance
  private static final double tolerance = 0.5;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedSparkMax motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedSparkMax motorFollower;

  private final RelativeEncoder encoderLeader;
  private final RelativeEncoder encoderFollower;
  private final SparkClosedLoopController controller;

  private final ElevatorFeedforward ffController;
  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  public Elevator(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake);

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
                  .allowedClosedLoopError(tolerance);

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

    ffController = new ElevatorFeedforward(kS, kG, kV, kA);
    trapezoidProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // make FF configuration tunable
    Tunable.bind("Elevator/FF", ffController);

    reset();
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
    state.position = getPosition();
    state.velocity = getVelocity();

    // this calculate method is doing at least two new allocations... :(
    state = trapezoidProfile.calculate(Constants.cyclePeriodSeconds, state, goal);
    final var ff = ffController.calculateWithVelocities(getVelocity(), state.velocity);

    controller.setReference(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public void reset() {
    encoderLeader.setPosition(0);
  }

  public void setPosition(double value) {
    setpoint = MathUtil.clamp(value, 0, maxTravel);
  }

  public boolean isAtPosition(double value) {
    return isAtPosition(value, tolerance);
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

  @Logged(name = "State/Position", importance = Importance.DEBUG)
  public double statePosition() {
    return state.position;
  }

  @Logged(name = "State/Velocity", importance = Importance.DEBUG)
  public double stateVelocity() {
    return state.velocity;
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
  private Tunable.DoubleValue maxVelocity = Tunable.value("Elevator/FF/MaxVelocity", kMaxVelocity, this::updateMaxVelocity);
  private Tunable.DoubleValue maxAcceleration = Tunable.value("Elevator/FF/MaxAcceleration", kMaxAcceleration, this::updateMaxAcceleration);
  // spotless:on
}
