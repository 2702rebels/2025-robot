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
  public final BitSet algae;

  /** Number of corals scored at level 1. */
  public byte coralsL1;

  /** Coral positions scored at level 2. */
  public final BitSet coralsL2;

  /** Coral positions scored at level 3. */
  public final BitSet coralsL3;

  /** Coral positions scored at level 4. */
  public final BitSet coralsL4;

  /**
   * Scoring target position encoded as "(level - 1) * 12 + index" or -1 if no target is specified.
   * Indices between 0 and 11 are reserved, 12 to 23 represent corals at level 2, and so on.
   */
  public byte target;

  public ReefState() {
    algae = new BitSet(6);
    coralsL1 = 0;
    coralsL2 = new BitSet(12);
    coralsL3 = new BitSet(12);
    coralsL4 = new BitSet(12);
    target = -1;
  }

  public ReefState(
      BitSet algae, byte coralsL1, BitSet coralsL2, BitSet coralsL3, BitSet coralsL4, byte target) {
    this.algae = algae;
    this.coralsL1 = coralsL1;
    this.coralsL2 = coralsL2;
    this.coralsL3 = coralsL3;
    this.coralsL4 = coralsL4;
    this.target = target;
  }

  /** ReefState struct for serialization. */
  public static final ReefStateStruct struct = new ReefStateStruct();
}
