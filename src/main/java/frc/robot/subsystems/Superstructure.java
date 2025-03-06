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
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kStowedHeight = 0;
  private static final double __kFloorHeight = 0;
  private static final double __kStationHeight = 28;
  private static final double __kL1Height = 16;
  private static final double __kL2Height = 31;
  private static final double __kL3Height = 47;
  private static final double __kL4Height = 64;
  private static final double __kL4SHeight = 46;
  private static final double __kL2AlgaeHeight = 0;
  private static final double __kL3AlgaeHeight = 0;
  private static final double __kNetAlgaeHeight = 66;

  // angles are in degrees [!]
  private static final double __kStowedAngle = 130;
  private static final double __kTransitionAngle = 80;
  private static final double __kStowedAlgaeAngle = 70;
  private static final double __kTransitionAlgaeAngle = 30;
  private static final double __kFloorAngle = 1;
  private static final double __kStationAngle = 36;
  private static final double __kL1Angle = 25;
  private static final double __kL2Angle = 4;
  private static final double __kL3Angle = 4;
  private static final double __kL4Angle = 80;
  private static final double __kL4SAngle = 80;
  private static final double __kL2AlgaeAngle = 0;
  private static final double __kL3AlgaeAngle = 0;

  private static final double __kBranchMargin = 4;
  private static final double __kBranchOffset = 8;

  private static final double kSafeEjectAngle = 50;
  private static final double kAngleTolerance = 2;
  private static final double kElevatorTolerance = 1;

  @Logged(name = "Position", importance = Importance.INFO)
  private Position position = Position.STOWED;

  // delay on the coral presence trigger to allow it fully
  // leave the mechanism before shutting down motors, etc.
  private static final double kHasCoralDelaySeconds = 0.4;

  // delay on the coral presence trigger (rising edge) to allow
  // motors to run longer when game piece is partially in
  private static final double kCoralIntakeDelaySeconds = 0.5;

  private final RobotState state;
  private final Elevator elevator;
  private final Pivot pivot;

  private final SparkLimitSwitch coralSensor;

  // since we don't currently have a sensor (limit switch) to detect the presence
  // of the algae, we instead rely on current sensing and setting this state manually
  private boolean hasAlgaeState;

  public Superstructure(Elevator elevator, Pivot pivot, Manipulator manipulator) {
    this.state = RobotState.getInstance();
    this.elevator = elevator;
    this.pivot = pivot;

    coralSensor = manipulator.getCoralPresenceSensor();

    // spotless:off
    canEject = new Trigger(() -> pivot.getAngle() <= kSafeEjectAngle);
    isStowed = new Trigger(() -> elevator.isAtPosition(kStowedHeight.get()) && pivot.isAtAngle(kStowedAngle.get()));
    canScoreL1 = new Trigger(() -> elevator.isAtPosition(kL1Height.get()) && pivot.isAtAngle(kL1Angle.get()));
    canScoreL2 = new Trigger(() -> elevator.isAtPosition(kL2Height.get()) && pivot.isAtAngle(kL2Angle.get()));
    canScoreL3 = new Trigger(() -> elevator.isAtPosition(kL3Height.get()) && pivot.isAtAngle(kL3Angle.get()));
    canScoreL4 = new Trigger(() -> elevator.isAtPosition(kL4Height.get()) && pivot.isAtAngle(kL4Angle.get()));
    // spotless:on

    // use higher tolerance on the intake, since the elevator/pivot is moved up
    // once the coral is consumed by the manipulator
    canFloorIntake =
        new Trigger(
            () ->
                elevator.isAtPosition(kFloorHeight.get(), 1)
                    && pivot.isAtAngle(kFloorAngle.get(), 10));

    canStationIntake =
        new Trigger(
            () ->
                elevator.isAtPosition(kStationHeight.get(), 1)
                    && pivot.isAtAngle(kStationAngle.get(), 10));

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

    canAlgaeIntake =
        new Trigger(
            () -> {
              switch (position) {
                case L2_ALGAE:
                  return elevator.isAtPosition(kL2AlgaeHeight.get())
                      && pivot.isAtAngle(kL2AlgaeAngle.get());
                case L3_ALGAE:
                  return elevator.isAtPosition(kL3AlgaeHeight.get())
                      && pivot.isAtAngle(kL3AlgaeAngle.get());
                default:
                  return false;
              }
            });

    // spotless:off
    hasCoral = new Trigger(coralSensor::isPressed).debounce(kHasCoralDelaySeconds, DebounceType.kFalling).debounce(kCoralIntakeDelaySeconds, DebounceType.kRising);
    hasAlgae = new Trigger(() -> hasAlgaeState);
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
    L2_ALGAE,
    L3_ALGAE,
  }

  /** Trigger for coral presence. */
  @Logged(name = "HasCoral", importance = Importance.INFO)
  public final Trigger hasCoral;

  /** Trigger for algae presence. */
  @Logged(name = "HasAlgae", importance = Importance.INFO)
  public final Trigger hasAlgae;

  /** Trigger for pivot in stowed position. */
  @Logged(name = "IsStowed", importance = Importance.INFO)
  public final Trigger isStowed;

  /** Trigger for pivot position safe for ejection. */
  @Logged(name = "CanEject", importance = Importance.INFO)
  public final Trigger canEject;

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

  /** Trigger for superstructure ready to intake algae (from the reef). */
  @Logged(name = "CanAlgaeIntake", importance = Importance.INFO)
  public final Trigger canAlgaeIntake;

  public Command setPosition(Position position) {
    return runOnce(
            () -> {
              this.position = position;
            })
        .withName("SetPosition");
  }

  public Command maintainPosition(Position position) {
    return run(() -> {
          this.position = position;
        })
        .withName("MaintainPosition");
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

  public boolean isDoneScoringL4() {
    return elevator.isAtPosition(kL4SHeight.get());
  }

  @Override
  public void periodic() {
    final var setpoint = getSetpoint();

    elevator.setPosition(setpoint.getFirst());
    pivot.setAngle(setpoint.getSecond());
  }

  /* Manually sets algae presence indicator. */
  public void setAlgaePresence(boolean value) {
    hasAlgaeState = value;
  }

  /** Returns the next setpoint for elevator and pivot. */
  private Pair<Double, Double> getSetpoint() {
    final var height = elevator.getPosition();
    final var profile = positionProfiles.get(position);

    final var targetHeight = profile.getFirst();
    var targetAngle = profile.getSecond();

    // override stowed angle if we think we have an algae
    if (position == Position.STOWED && hasAlgaeState) {
      targetAngle = kStowedAlgaeAngle.get();
    }

    // we should only transition to L4 scored from L4, so no additional
    // logic is needed, just command new height (and angle)
    if (position == Position.L4_SCORED) {
      return Pair.of(targetHeight, targetAngle);
    }

    final var branchHeight = targetHeight - kBranchOffset.get();
    final var transitionAngle =
        hasAlgaeState ? kTransitionAlgaeAngle.get() : kTransitionAngle.get();

    // if the mechanism is above the requested position, then we should first go to
    // the requested profile branching point while maintaining pivot at the transition angle,
    // e.g. we are at L3 and need to go to L1
    //
    // use safety margin when testing to prevent falling into this case when we are
    // trying to reach the profile height from below and overshooting slightly
    if (height > targetHeight + kBranchMargin.get()) {
      return Pair.of(branchHeight, transitionAngle);
    }

    // otherwise elevate the pivot to the desired height, but do not rotate the pivot
    // until we are above the branching point

    // safety rules
    // do not allow pivot to stow unless elevator height is near zero
    // do not allow elevator to go up unless pivot is at transition angle or less
    final var pivotAngle = pivot.getAngle();
    final var desiredAngle = height >= branchHeight ? targetAngle : transitionAngle;

    return Pair.of(
        pivotAngle <= transitionAngle + kAngleTolerance ? targetHeight : 0,
        height > kElevatorTolerance ? Math.min(desiredAngle, transitionAngle) : desiredAngle);
  }

  private static Map<Position, Pair<Double, Double>> positionProfiles = new HashMap<>();

  private static void updatePositionProfiles(double _value) {
    // we prefer to cache profiles for each position since these do not change once tuned,
    // but we are constantly fetching them in the periodic to determine where mechanism must go,
    // caching prevents unnecessary allocations and improves lookup time

    // spotless:off
    positionProfiles.put(Position.STOWED, Pair.of(kStowedHeight.get(), kStowedAngle.get()));
    positionProfiles.put(Position.FLOOR, Pair.of(kFloorHeight.get(), kFloorAngle.get()));
    positionProfiles.put(Position.STATION, Pair.of(kStationHeight.get(), kStationAngle.get()));
    positionProfiles.put(Position.L1, Pair.of(kL1Height.get(), kL1Angle.get()));
    positionProfiles.put(Position.L2, Pair.of(kL2Height.get(), kL2Angle.get()));
    positionProfiles.put(Position.L3, Pair.of(kL3Height.get(), kL3Angle.get()));
    positionProfiles.put(Position.L4, Pair.of(kL4Height.get(), kL4Angle.get()));
    positionProfiles.put(Position.L4_SCORED, Pair.of(kL4SHeight.get(), kL4SAngle.get()));
    positionProfiles.put(Position.L2_ALGAE, Pair.of(kL2AlgaeHeight.get(), kL2AlgaeAngle.get()));
    positionProfiles.put(Position.L3_ALGAE, Pair.of(kL3AlgaeHeight.get(), kL3AlgaeAngle.get()));
    // spotless:on
  }

  private static Tunable.DoubleValue profileValue(String name, double defaultValue) {
    return Tunable.value(
        "Superstructure/" + name, defaultValue, Superstructure::updatePositionProfiles);
  }

  // spotless:off
  private static Tunable.DoubleValue kStowedAngle = profileValue("Stowed/Angle", __kStowedAngle);
  private static Tunable.DoubleValue kStowedHeight = profileValue("Stowed/Height", __kStowedHeight);
  private static Tunable.DoubleValue kFloorAngle = profileValue("Floor/Angle", __kFloorAngle);
  private static Tunable.DoubleValue kFloorHeight = profileValue("Floor/Height", __kFloorHeight);
  private static Tunable.DoubleValue kStationAngle = profileValue("Station/Angle", __kStationAngle);
  private static Tunable.DoubleValue kStationHeight = profileValue("Station/Height", __kStationHeight);
  private static Tunable.DoubleValue kL1Angle = profileValue("L1/Angle", __kL1Angle);
  private static Tunable.DoubleValue kL1Height = profileValue("L1/Height", __kL1Height);
  private static Tunable.DoubleValue kL2Angle = profileValue("L2/Angle", __kL2Angle);
  private static Tunable.DoubleValue kL2Height = profileValue("L2/Height", __kL2Height);
  private static Tunable.DoubleValue kL3Angle = profileValue("L3/Angle", __kL3Angle);
  private static Tunable.DoubleValue kL3Height = profileValue("L3/Height", __kL3Height);
  private static Tunable.DoubleValue kL4Angle = profileValue("L4/Angle", __kL4Angle);
  private static Tunable.DoubleValue kL4Height = profileValue("L4/Height", __kL4Height);
  private static Tunable.DoubleValue kL4SAngle = profileValue("L4Scored/Angle", __kL4SAngle);
  private static Tunable.DoubleValue kL4SHeight = profileValue("L4Scored/Height", __kL4SHeight);
  private static Tunable.DoubleValue kL2AlgaeAngle = profileValue("L2Algae/Angle", __kL2AlgaeAngle);
  private static Tunable.DoubleValue kL2AlgaeHeight = profileValue("L2Algae/Height", __kL2AlgaeHeight);
  private static Tunable.DoubleValue kL3AlgaeAngle = profileValue("L3Algae/Angle", __kL3AlgaeAngle);
  private static Tunable.DoubleValue kL3AlgaeHeight = profileValue("L3Algae/Height", __kL3AlgaeHeight);
  private static Tunable.DoubleValue kNetAlgaeHeight = profileValue("NetAlgae/Height", __kNetAlgaeHeight);
  private static Tunable.DoubleValue kBranchMargin = profileValue("BranchMargin", __kBranchMargin);
  private static Tunable.DoubleValue kBranchOffset = profileValue("BranchOffset", __kBranchOffset);
  private static Tunable.DoubleValue kTransitionAngle = profileValue("TransitionAngle", __kTransitionAngle);
  private static Tunable.DoubleValue kTransitionAlgaeAngle = profileValue("TransitionAlgaeAngle", __kTransitionAlgaeAngle);
  private static Tunable.DoubleValue kStowedAlgaeAngle = profileValue("StowedAlgaeAngle", __kStowedAlgaeAngle);
  // spotless:on

  static {
    updatePositionProfiles(0); // value is ignored
  }
}
