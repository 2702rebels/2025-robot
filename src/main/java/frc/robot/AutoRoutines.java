package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutines {
  private final AutoFactory factory;
  private final RobotState state;

  public AutoRoutines(AutoFactory factory) {
    this.factory = factory;
    this.state = RobotState.getInstance();
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = factory.newRoutine("SimplePath Auto");
    final AutoTrajectory simplePath = routine.trajectory("SimplePath");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  public AutoRoutine scoreL2() {
    final AutoRoutine routine = factory.newRoutine("ScoreL2");

    routine.active().onTrue(state.scoreAuto(2));
    return routine;
  }

  public AutoRoutine scoreL3() {
    final AutoRoutine routine = factory.newRoutine("ScoreL3");

    routine.active().onTrue(state.scoreAuto(3));
    return routine;
  }

  public AutoRoutine scoreL4() {
    final AutoRoutine routine = factory.newRoutine("ScoreL4");

    routine.active().onTrue(state.scoreAuto(4));
    return routine;
  }

  public AutoRoutine naive(String name, Command command) {
    final AutoRoutine routine = factory.newRoutine(name);

    routine.active().onTrue(command);
    return routine;
  }
}
