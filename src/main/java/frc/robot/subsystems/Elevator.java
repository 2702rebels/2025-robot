package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  private static final double kP = 1;

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
            .smartCurrentLimit(60)
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .signals
            .primaryEncoderPositionAlwaysOn(true);

    motorLeader =
        new LoggedSparkMax(
            leaderCanID,
            (config) -> {
              config
                  .apply(commonConfig)
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(kP);
            });

    motorFollower =
        new LoggedSparkMax(
            followerCanID,
            (config) -> {
              // follow leader's output (non-inverted)
              config.apply(commonConfig).follow(motorLeader);
            });

    encoderLeader = motorLeader.getEncoder();
    encoderFollower = motorFollower.getEncoder();
    controller = motorLeader.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Elevator", motorLeader, ClosedLoopSlot.kSlot0);
  }

  /** Trigger for elevator position ready for floor intake */
  @Logged(name = "CanFloorIntake", importance = Importance.INFO)
  public final Trigger canFloorIntake = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  /** Trigger for elevator position ready for station intake */
  @Logged(name = "CanStationIntake", importance = Importance.INFO)
  public final Trigger canStationIntake = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  /** Trigger for elevator position ready for scoring at L1 */
  @Logged(name = "CanScoreL1", importance = Importance.INFO)
  public final Trigger canScoreL1 = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  /** Trigger for elevator position ready for scoring at L2 */
  @Logged(name = "CanScoreL2", importance = Importance.INFO)
  public final Trigger canScoreL2 = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  /** Trigger for elevator position ready for scoring at L3 */
  @Logged(name = "CanScoreL3", importance = Importance.INFO)
  public final Trigger canScoreL3 = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  /** Trigger for elevator position ready for scoring at L4 */
  @Logged(name = "CanScoreL4", importance = Importance.INFO)
  public final Trigger canScoreL4 = new Trigger(() -> getLeaderPosition() > 0); // TODO: value

  public void reset() {
    encoderLeader.setPosition(0);
  }

  public void setPosition(double value) {
    controller.setReference(value, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
}
