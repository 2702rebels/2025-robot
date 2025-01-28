package frc.robot.lib.struct;

import edu.wpi.first.util.struct.Struct;
import frc.robot.lib.ReefState;
import java.nio.ByteBuffer;
import java.util.BitSet;

public class ReefStateStruct implements Struct<ReefState> {
  @Override
  public Class<ReefState> getTypeClass() {
    return ReefState.class;
  }

  @Override
  public String getTypeName() {
    return "ReefState";
  }

  @Override
  public int getSize() {
    return kSizeInt16 * 3 + kSizeInt8 * 3 + kSizeBool;
  }

  @Override
  public String getSchema() {
    return "uint8 algae;uint8 coralsL1;uint16 coralsL2;uint16 coralsL3;uint16 coralsL4;int8 target;bool locked;";
  }

  @Override
  public ReefState unpack(ByteBuffer bb) {
    return new ReefState(
        BitSet.valueOf(bb.slice(0, 1)),
        bb.get(1),
        BitSet.valueOf(bb.slice(2, 2)),
        BitSet.valueOf(bb.slice(4, 2)),
        BitSet.valueOf(bb.slice(6, 2)),
        bb.get(8),
        bb.get(9) == 1);
  }

  @Override
  public void pack(ByteBuffer bb, ReefState value) {
    bb.put(toByteArray(value.algae, 6));
    bb.put(value.coralsL1);
    bb.put(toByteArray(value.coralsL2, 12));
    bb.put(toByteArray(value.coralsL3, 12));
    bb.put(toByteArray(value.coralsL4, 12));
    bb.put(value.target);
    bb.put(value.locked ? (byte) 1 : (byte) 0);
  }

  /**
   * Returns little-endian byte representation of the bitset with the fixed length regardless of the
   * number of bits actually set.
   *
   * <p>Note that built-in BitSet.toByteArray method always packs the bitset into the smallest
   * possible representation, i.e. bitset with all false bits is represented by an empty array,
   * which is not what we want here.
   */
  private static byte[] toByteArray(BitSet bitset, int length) {
    final var bytes = new byte[(length >>> 3) + 1];

    byte b = 0;
    for (var i = 0; i < length; ++i) {
      final var bit = i % 8;
      if (bitset.get(i)) {
        b |= (1 << bit);
      }

      // little-endian representation for multi-byte
      if (bit == 7) {
        bytes[i / 8] = b;
        b = 0;
      }
    }

    if (b != 0) {
      bytes[bytes.length - 1] = b;
    }

    return bytes;
  }
}
