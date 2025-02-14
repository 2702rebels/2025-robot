package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.Tunable;
import java.util.HashMap;
import java.util.Map;

/**
 * Combines elevator and pivot (arm) since we want a coupled control of both during motion to the
 * desired position.
 */
public final class SuperStructure extends SubsystemBase {
  private boolean isCoralPresent;
  private boolean isAlgaePresent;

  private Position position;

  /** Mechanism positions */
  public enum Position {
    STOWED,
    FLOOR,
    STATION,
    L1,
    L2,
    L3,
    L4,
  }

  /** Trigger for coral presence. */
  public final Trigger hasCoral = new Trigger(() -> isCoralPresent);

  /** Trigger for algae presence. */
  public final Trigger hasAlgae = new Trigger(() -> isAlgaePresent);

  /** Sets the desired position. */
  public void setPosition(Position position) {
    this.position = position;
  }

  @Override
  public void periodic() {
    // TODO: use getSetpoint() to command elevator and arm position
  }

  /** Returns current height of the elevator. */
  private double getElevatorHeight() {
    return 0;
  }

  /** Returns the next setpoint for elevator and arm. */
  private Pair<Double, Double> getSetpoint() {
    final var height = getElevatorHeight();
    final var profile = positionProfiles.get(position);

    final var targetHeight = profile.getFirst();
    final var targetAngle = profile.getSecond();
    final var branchHeight = targetHeight - branchOffset.get();

    // if the mechanism is above the requested position, then we should first go to
    // the requested profile branching point while maintaining arm at the transition angle,
    // e.g. we are at L3 and need to go to L1
    //
    // use safety margin when testing to prevent falling into this case when we are
    // trying to reach the profile height from below and overshooting slightly
    if (height > targetHeight + branchMargin.get()) {
      return Pair.of(branchHeight, transitionAngle.get());
    }

    // otherwise elevate the arm to the desired height, but do not rotate the arm
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
    // spotless:on
  }

  private static Tunable.DoubleValue profileValue(String name, double defaultValue) {
    return Tunable.value(
        "Superstructure/" + name, defaultValue, SuperStructure::updatePositionProfiles);
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
  private static Tunable.DoubleValue branchMargin = profileValue("BranchMargin", 0);
  private static Tunable.DoubleValue branchOffset = profileValue("BranchOffset", 0);
  private static Tunable.DoubleValue transitionAngle = profileValue("TransitionAngle", 0);
  // spotless:on
}
