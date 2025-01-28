package frc.robot.lib;

import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.lib.struct.ReefStateStruct;
import java.util.BitSet;

/**
 * A mutable state representation of the reef scoring elements and target.
 *
 * <p>Coral array indices start from the face closest to the alliance wall on the left side and go
 * counterclockwise, corresponding to labels A to L in the game manual.
 *
 * <p>Algae array indices start from the face closest to the alliance wall and go counterclockwise,
 * corresponding to sides AB, CD, EF, GH, IJ, KL.
 */
public class ReefState implements StructSerializable {
  /* Algae presence. */
  public BitSet algae;

  /** Number of corals scored at level 1. */
  public byte coralsL1;

  /** Coral positions scored at level 2. */
  public BitSet coralsL2;

  /** Coral positions scored at level 3. */
  public BitSet coralsL3;

  /** Coral positions scored at level 4. */
  public BitSet coralsL4;

  /**
   * Scoring target position encoded as "(level - 1) * 12 + index" or -1 if no target is specified.
   * Indices between 0 and 11 are reserved, 12 to 23 represent corals at level 2, and so on.
   */
  public byte target;

  /** Indicates that target is locked by the operator. */
  public boolean locked;

  /** Determines whether target position is marked as scored. */
  public boolean isTargetScored() {
    if (this.target < 12) {
      return false;
    }

    final var level = (this.target / 12);
    final var index = this.target % 12;
    switch (level) {
      case 1:
        return coralsL2.get(index);
      case 2:
        return coralsL3.get(index);
      case 3:
        return coralsL4.get(index);
    }
    return false;
  }

  public ReefState() {
    // algae is preloaded
    algae = new BitSet(6);
    algae.set(0, 6);

    coralsL1 = 0;
    coralsL2 = new BitSet(12);
    coralsL3 = new BitSet(12);
    coralsL4 = new BitSet(12);
    target = -1;
    locked = false;
  }

  public ReefState(
      BitSet algae,
      byte coralsL1,
      BitSet coralsL2,
      BitSet coralsL3,
      BitSet coralsL4,
      byte target,
      boolean locked) {
    this.algae = algae;
    this.coralsL1 = coralsL1;
    this.coralsL2 = coralsL2;
    this.coralsL3 = coralsL3;
    this.coralsL4 = coralsL4;
    this.target = target;
    this.locked = locked;
  }

  /** Returns bitset for the specified level index (1...3). */
  public BitSet getCorals(int levelIndex) {
    switch (levelIndex) {
      case 1:
        return coralsL2;
      case 2:
        return coralsL3;
      case 3:
        return coralsL4;
      default:
        throw new IndexOutOfBoundsException();
    }
  }

  /** Resets this instance to the default field state. */
  public void reset() {
    algae.set(0, 6);
    coralsL1 = 0;
    coralsL2.clear();
    coralsL3.clear();
    coralsL4.clear();
    target = -1;
    locked = false;
  }

  /** ReefState struct for serialization. */
  public static final ReefStateStruct struct = new ReefStateStruct();
}
