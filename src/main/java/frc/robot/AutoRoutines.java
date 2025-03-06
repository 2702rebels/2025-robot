package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

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

  public AutoRoutine naive(String name, Command command) {
    final AutoRoutine routine = factory.newRoutine(name);

    routine.active().onTrue(command);
    return routine;
  }
}
