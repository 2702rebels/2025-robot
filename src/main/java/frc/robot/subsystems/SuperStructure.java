package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.lib.Tunable;
import java.util.HashMap;
import java.util.Map;

/**
 * Combines elevator and pivot since we want a coupled control of both during motion to the desired
 * position.
 */
public final class Superstructure extends SubsystemBase {
  @Logged(name = "Position", importance = Importance.INFO)
  private Position position = Position.STOWED;

  // delay on the game piece presence trigger to allow it fully
  // leave the mechanism before shutting down motors, etc.
  private static final double kHasGamePieceDelaySeconds = 0.2;

  private final RobotState state;
  private final Elevator elevator;
  private final Pivot pivot;

  private final SparkLimitSwitch coralSensor;
  private final SparkLimitSwitch algaeSensor;

  public Superstructure(Elevator elevator, Pivot pivot, Manipulator manipulator) {
    this.state = RobotState.getInstance();
    this.elevator = elevator;
    this.pivot = pivot;

    coralSensor = manipulator.getCoralPresenceSensor();
    algaeSensor = pivot.getAlgaePresenceSensor();

    canFloorIntake = elevator.canFloorIntake.and(pivot.canFloorIntake);
    canStationIntake = elevator.canStationIntake.and(pivot.canStationIntake);
    canScoreL1 = elevator.canScoreL1.and(pivot.canScoreL1);
    canScoreL2 = elevator.canScoreL2.and(pivot.canScoreL2);
    canScoreL3 = elevator.canScoreL3.and(pivot.canScoreL3);
    canScoreL4 = elevator.canScoreL4.and(pivot.canScoreL4);

    canScoreAuto =
        new Trigger(
            () -> {
              switch (state.getScoreLevel()) {
                case 1:
                  return canScoreL1.getAsBoolean();
                case 2:
                  return canScoreL2.getAsBoolean();
                case 3:
                  return canScoreL3.getAsBoolean();
                case 4:
                  return canScoreL4.getAsBoolean();
                default:
                  return false;
              }
            });

    // spotless:off
    hasCoral = new Trigger(coralSensor::isPressed).debounce(kHasGamePieceDelaySeconds, DebounceType.kFalling);
    hasAlgae = new Trigger(algaeSensor::isPressed).debounce(kHasGamePieceDelaySeconds, DebounceType.kFalling);
    // spotless:on
  }

  /** Mechanism positions */
  public enum Position {
    STOWED,
    FLOOR,
    STATION,
    L1,
    L2,
    L3,
    L4,
    L4_SCORED,
  }

  /** Trigger for coral presence. */
  @Logged(name = "HasCoral", importance = Importance.INFO)
  public final Trigger hasCoral;

  /** Trigger for algae presence. */
  @Logged(name = "HasAlgae", importance = Importance.INFO)
  public final Trigger hasAlgae;

  /** Trigger for superstructure ready for floor intake */
  @Logged(name = "CanFloorIntake", importance = Importance.INFO)
  public final Trigger canFloorIntake;

  /** Trigger for superstructure ready for station intake */
  @Logged(name = "CanStationIntake", importance = Importance.INFO)
  public final Trigger canStationIntake;

  /** Trigger for superstructure ready for scoring at L1 */
  @Logged(name = "CanScoreL1", importance = Importance.INFO)
  public final Trigger canScoreL1;

  /** Trigger for superstructure ready for scoring at L2 */
  @Logged(name = "CanScoreL2", importance = Importance.INFO)
  public final Trigger canScoreL2;

  /** Trigger for superstructure ready for scoring at L3 */
  @Logged(name = "CanScoreL3", importance = Importance.INFO)
  public final Trigger canScoreL3;

  /** Trigger for superstructure ready for scoring at L4 */
  @Logged(name = "CanScoreL4", importance = Importance.INFO)
  public final Trigger canScoreL4;

  /** Trigger for superstructure ready for scoring at auto scoring level. */
  @Logged(name = "CanScoreAuto", importance = Importance.INFO)
  public final Trigger canScoreAuto;

  /** Trigger for superstructure directed to score at L4. */
  @Logged(name = "TargetsL4", importance = Importance.INFO)
  public final Trigger targetsL4 = new Trigger(() -> position == Position.L4);

  public Command setPosition(Position position) {
    return runOnce(
            () -> {
              this.position = position;
            })
        .withName("SetPosition");
  }

  public Command setAutoScoringPosition() {
    return runOnce(
            () -> {
              switch (state.getScoreLevel()) {
                case 1:
                  this.position = Position.L1;
                  break;
                case 2:
                  this.position = Position.L2;
                  break;
                case 3:
                  this.position = Position.L3;
                  break;
                case 4:
                  this.position = Position.L4;
                  break;
              }
            })
        .withName("SetAutoScoringPosition");
  }

  @Override
  public void periodic() {
    final var setpoint = getSetpoint();

    elevator.setPosition(setpoint.getFirst());
    pivot.setAngle(setpoint.getSecond());
  }

  /** Returns current height of the elevator. */
  private double getElevatorHeight() {
    return elevator.getLeaderPosition();
  }

  /** Returns the next setpoint for elevator and pivot. */
  private Pair<Double, Double> getSetpoint() {
    final var height = getElevatorHeight();
    final var profile = positionProfiles.get(position);

    final var targetHeight = profile.getFirst();
    final var targetAngle = profile.getSecond();

    // we should only transition to L4 scored from L4, so no additional
    // logic is needed, just command new height (and angle)
    if (position == Position.L4_SCORED) {
      return Pair.of(targetHeight, targetAngle);
    }

    final var branchHeight = targetHeight - branchOffset.get();

    // if the mechanism is above the requested position, then we should first go to
    // the requested profile branching point while maintaining pivot at the transition angle,
    // e.g. we are at L3 and need to go to L1
    //
    // use safety margin when testing to prevent falling into this case when we are
    // trying to reach the profile height from below and overshooting slightly
    if (height > targetHeight + branchMargin.get()) {
      return Pair.of(branchHeight, transitionAngle.get());
    }

    // otherwise elevate the pivot to the desired height, but do not rotate the pivot
    // until we are above the branching point
    return Pair.of(targetHeight, height >= branchHeight ? targetAngle : transitionAngle.get());
  }

  private static Map<Position, Pair<Double, Double>> positionProfiles = new HashMap<>();

  private static void updatePositionProfiles(double _value) {
    // we prefer to cache profiles for each position since these do not change once tuned,
    // but we are constantly fetching them in the periodic to determine where mechanism must go,
    // caching prevents unnecessary allocations and improves lookup time

    // spotless:off
    positionProfiles.put(Position.STOWED, Pair.of(stowedHeight.get(), stowedAngle.get()));
    positionProfiles.put(Position.FLOOR, Pair.of(floorHeight.get(), floorAngle.get()));
    positionProfiles.put(Position.STATION, Pair.of(stationHeight.get(), stationAngle.get()));
    positionProfiles.put(Position.L1, Pair.of(l1Height.get(), l1Angle.get()));
    positionProfiles.put(Position.L2, Pair.of(l2Height.get(), l2Angle.get()));
    positionProfiles.put(Position.L3, Pair.of(l3Height.get(), l3Angle.get()));
    positionProfiles.put(Position.L4, Pair.of(l4Height.get(), l4Angle.get()));
    positionProfiles.put(Position.L4_SCORED, Pair.of(l4SHeight.get(), l4SAngle.get()));
    // spotless:on
  }

  private static Tunable.DoubleValue profileValue(String name, double defaultValue) {
    return Tunable.value(
        "Superstructure/" + name, defaultValue, Superstructure::updatePositionProfiles);
  }

  // spotless:off
  private static Tunable.DoubleValue stowedAngle = profileValue("Stowed/Angle", 0);
  private static Tunable.DoubleValue stowedHeight = profileValue("Stowed/Height", 0);
  private static Tunable.DoubleValue floorAngle = profileValue("Floor/Angle", 0);
  private static Tunable.DoubleValue floorHeight = profileValue("Floor/Height", 0);
  private static Tunable.DoubleValue stationAngle = profileValue("Station/Angle", 0);
  private static Tunable.DoubleValue stationHeight = profileValue("Station/Height", 0);
  private static Tunable.DoubleValue l1Angle = profileValue("L1/Angle", 0);
  private static Tunable.DoubleValue l1Height = profileValue("L1/Height", 0);
  private static Tunable.DoubleValue l2Angle = profileValue("L2/Angle", 0);
  private static Tunable.DoubleValue l2Height = profileValue("L2/Height", 0);
  private static Tunable.DoubleValue l3Angle = profileValue("L3/Angle", 0);
  private static Tunable.DoubleValue l3Height = profileValue("L3/Height", 0);
  private static Tunable.DoubleValue l4Angle = profileValue("L4/Angle", 0);
  private static Tunable.DoubleValue l4Height = profileValue("L4/Height", 0);
  private static Tunable.DoubleValue l4SAngle = profileValue("L4Scored/Angle", 0);
  private static Tunable.DoubleValue l4SHeight = profileValue("L4Scored/Height", 0);
  private static Tunable.DoubleValue branchMargin = profileValue("BranchMargin", 0);
  private static Tunable.DoubleValue branchOffset = profileValue("BranchOffset", 0);
  private static Tunable.DoubleValue transitionAngle = profileValue("TransitionAngle", 0);
  // spotless:on

  static {
    updatePositionProfiles(0); // value is ignored
  }
}
