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
    return kSizeInt16 * 3 + kSizeInt8 * 3;
  }

  @Override
  public String getSchema() {
    return "uint8 algae;uint8 coralsL1;uint16 coralsL2;uint16 coralsL3;uint16 coralsL4;int8 target;";
  }

  @Override
  public ReefState unpack(ByteBuffer bb) {
    return new ReefState(
        BitSet.valueOf(bb.slice(0, 1)),
        bb.get(),
        BitSet.valueOf(bb.slice(2, 2)),
        BitSet.valueOf(bb.slice(4, 2)),
        BitSet.valueOf(bb.slice(6, 2)),
        bb.get());
  }

  @Override
  public void pack(ByteBuffer bb, ReefState value) {
    // note that BitSet.toByteArray() returns little-endian representation
    // which is the same as required by the protocol
    bb.put(value.algae.toByteArray());
    bb.put(value.coralsL1);
    bb.put(value.coralsL2.toByteArray());
    bb.put(value.coralsL3.toByteArray());
    bb.put(value.coralsL4.toByteArray());
    bb.put(value.target);
  }
}
