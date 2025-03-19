package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.Intent;
import frc.robot.subsystems.Superstructure;

public class AutoRoutines {
  private final AutoFactory factory;
  private final RobotState state;
  private final Superstructure superstructure;

  public AutoRoutines(AutoFactory factory, Superstructure superstructure) {
    this.factory = factory;
    this.superstructure = superstructure;
    this.state = RobotState.getInstance();
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = factory.newRoutine("SimplePath Auto");
    final AutoTrajectory simplePath = routine.trajectory("SimplePath");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  private int getLocationIndex(char c) {
    return c - 'A';
  }

  public AutoRoutine scoreL4x(char c) {
    final AutoRoutine routine = factory.newRoutine("Score L4 (" + c + ")");

    routine.active().onTrue(state.scoreAutoL4(getLocationIndex(c)));
    return routine;
  }

  public AutoRoutine scoreL4IL() {
    final AutoRoutine routine = factory.newRoutine("Score L4 (I,L)");

    final AutoTrajectory trajIto3 = routine.trajectory("Ito3");
    final AutoTrajectory traj3toLeft = routine.trajectory("3toLeft");

    routine.active().onTrue(state.scoreAutoL4(getLocationIndex('I')));

    state
        .idle
        .and(() -> state.getAutoScoringStep() == 1)
        .and(superstructure.hasCoral.negate())
        .onTrue(trajIto3.cmd());

    // trajIto3.atTime("IntakeDeploy").onTrue(state.setIntent(Intent.kCoralFloorIntake));
    trajIto3.atTime(0.9).onTrue(state.setIntent(Intent.kCoralFloorIntake));

    // if we have failed to intake the piece, then reset intent and chill
    // otherwise, reposition for the next scoring leg
    trajIto3.done().and(superstructure.hasCoral).onTrue(traj3toLeft.cmd());
    trajIto3.done().and(superstructure.hasCoral.negate()).onTrue(state.setIntent(Intent.kNone));

    traj3toLeft
        .done()
        .and(superstructure.hasCoral)
        .onTrue(state.scoreAutoL4(getLocationIndex('L')));

    return routine;
  }

  public AutoRoutine naive(String name, Command command) {
    final AutoRoutine routine = factory.newRoutine(name);

    routine.active().onTrue(command);
    return routine;
  }
}
