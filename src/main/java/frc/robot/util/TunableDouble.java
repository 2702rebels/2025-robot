package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/** Numeric (double) value that can be tuned via NetworkTables client. */
public class TunableDouble implements DoubleSupplier {
  private static final String tableKey = "/Tunable";

  private final DoubleEntry entry;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new tunable number with the default value.
   *
   * @param dashboardKey Key
   * @param defaultValue Default value
   */
  public TunableDouble(String key, double defaultValue) {
    this.entry =
        NetworkTableInstance.getDefault()
            .getDoubleTopic(tableKey + "/" + key)
            .getEntry(defaultValue);
  }

  /**
   * Get the current value.
   *
   * @return The current value
   */
  public double get() {
    return this.entry.get();
  }

  /**
   * Sets the current value.
   *
   * @param value The current value.
   */
  public void set(double value) {
    this.entry.set(value);
  }

  /**
   * Checks whether the number has changed since our last check.
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    final var currentValue = get();
    final var lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the numbers have changed.
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunable All tunable numbers to check
   */
  public static void ifChanged(int id, Consumer<double[]> action, TunableDouble... tunable) {
    if (Arrays.stream(tunable).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunable).mapToDouble(TunableDouble::get).toArray());
    }
  }

  /** Runs action if any of the tunable numbers have changed */
  public static void ifChanged(int id, Runnable action, TunableDouble... tunable) {
    ifChanged(id, values -> action.run(), tunable);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
