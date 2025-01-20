package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
  private final AutoFactory factory;

  public AutoRoutines(AutoFactory factory) {
    this.factory = factory;
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = factory.newRoutine("SimplePath Auto");
    final AutoTrajectory simplePath = routine.trajectory("SimplePath");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }
}
