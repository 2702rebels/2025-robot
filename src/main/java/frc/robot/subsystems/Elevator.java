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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 0.008;
  private static final double __kD = 0;

  private static final double __kG0 = 0;
  private static final double __kG1 = 0.3;
  private static final double __kV = 0.075;
  private static final double __kMaxVelocity = 100; // inch/sec
  private static final double __kMaxAcceleration = 400; // inch/sec.sq

  // conversion factor between rotation and  linear movement
  // 3" wheel diameter, 8:1 reduction ratio
  private static final double conversionFactor = Math.PI * 3 / 8;

  // maximum safe travel
  private static final double maxTravel = 66;

  // tolerance (inches)
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

  private TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State state = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

  public Elevator(int leaderCanID, int followerCanID) {
    final var commonConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(50)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake);

    final var signalsConfig =
        new SignalsConfig()
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(10)
            .primaryEncoderVelocityPeriodMs(10);

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
                  .p(__kP)
                  .d(__kD)
                  .maxMotion
                  .maxVelocity(__kMaxVelocity)
                  .maxAcceleration(__kMaxAcceleration)
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

    trapezoidProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(kMaxVelocity.get(), kMaxAcceleration.get()));

    reset();
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
    goal.position = setpoint; // goal velocity is always zero!
    state.position = getPosition();
    state.velocity = getVelocity();

    // this calculate method is doing at least two new allocations... :(
    state = trapezoidProfile.calculate(Constants.cyclePeriodSeconds, state, goal);

    // elevator's effective weight changes as it extends higher, since
    // more sections need to be supported, this requires different kG
    // values to be applied when elevator is near ground vs. extended
    //
    // kG0 - applied when elevator is near zero
    // kG1 - applied otherwise
    //
    // we don't use ElevatorFeedforward, instead we just use the basic FF
    // formula directly here since we have no kS nor kA components
    //
    // we don't apply velocity component (kV) when elevator is at setpoint
    // to reduce jittering, relying instead of kG and PID controls

    final var gravityFF = getPosition() < 16 ? kG0.get() : kG1.get(); // kG0 below 16"
    final var velocityFF = isAtPosition(setpoint) ? 0 : state.velocity * kV.get();

    controller.setReference(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityFF + velocityFF);
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
  private Tunable.DoubleValue kG0 = Tunable.value("Elevator/FF/kG0", __kG0);
  private Tunable.DoubleValue kG1 = Tunable.value("Elevator/FF/kG1", __kG1);
  private Tunable.DoubleValue kV = Tunable.value("Elevator/FF/kV", __kV);
  private Tunable.DoubleValue kMaxVelocity = Tunable.value("Elevator/FF/MaxVelocity", __kMaxVelocity, this::updateMaxVelocity);
  private Tunable.DoubleValue kMaxAcceleration = Tunable.value("Elevator/FF/MaxAcceleration", __kMaxAcceleration, this::updateMaxAcceleration);
  // spotless:on
}
