package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.ReefState;
import java.util.BitSet;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private static final int RP_CORALS_PER_LEVEL = 5;
  private static final int CORALS_PER_LEVEL = 12;

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  /*
   * ReefState to use with Reef widgets
   *
   * We are "mimicking" sendable data structure here (by including .type field)
   * since the widget requires two separate topics - one for sending operator's input,
   * and one for representing the robot's perspective.
   *
   * The robot code is responsible for merging the operator's and robot's perspective
   * into a consistent state. This approach avoids the problem of "missing updates"
   * that are possible if both sides were to modify the same data structure concurrently.
   */
  private final NetworkTable reefStateTable = nt.getTable("RobotState/ReefState");
  private final StringPublisher reefStateTypePublisher =
      reefStateTable.getStringTopic(".type").publish();
  private final StructPublisher<ReefState> reefStateRobotPublisher;
  private final StructSubscriber<ReefState> reefStateOperatorSubscriber;

  private final SendableChooser<ScoringStrategy> scoringStrategyChooser = new SendableChooser<>();

  private RobotState() {
    reefState = new ReefState();

    scoringStrategyChooser.setDefaultOption("Auto", ScoringStrategy.AUTO);
    scoringStrategyChooser.addOption("Ranking Point", ScoringStrategy.RP);
    scoringStrategyChooser.addOption("Score", ScoringStrategy.SCORE);

    reefStateRobotPublisher = reefStateTable.getStructTopic("robot", ReefState.struct).publish();
    reefStateOperatorSubscriber =
        reefStateTable
            .getStructTopic("operator", ReefState.struct)
            .subscribe(new ReefState(), PubSubOption.sendAll(false));

    SmartDashboard.putData("Scoring Strategy", scoringStrategyChooser);

    reefStateTypePublisher.setDefault("ReefState");
    reefStateRobotPublisher.setDefault(new ReefState());
  }

  public static enum ScoringStrategy {
    /** Automatic based on the match type (qualification / playoff) */
    AUTO,

    /** Meet RP requirement */
    RP,

    /** Maximize total score */
    SCORE,
  }

  /** Scoring strategy for the reef */
  public ScoringStrategy scoringStrategy = ScoringStrategy.AUTO;

  /** Reef state (robot side) */
  public final ReefState reefState;

  /** Synchronizes operator's input with the robot's perspective. */
  private void pollReefState() {
    final var values = reefStateOperatorSubscriber.readQueueValues();
    if (values.length == 0) return;

    for (var i = 0; i < values.length; ++i) {
      final var value = values[i];

      // set the robot's perspective to the operator's input
      reefState.algae = (BitSet) value.algae.clone();
      reefState.coralsL1 = value.coralsL1;
      reefState.coralsL2 = (BitSet) value.coralsL2.clone();
      reefState.coralsL3 = (BitSet) value.coralsL3.clone();
      reefState.coralsL4 = (BitSet) value.coralsL4.clone();
      reefState.target = value.target;
      reefState.locked = value.locked;
    }

    // if target position is marked as scored, remove the target and the lock
    if (reefState.isTargetScored()) {
      reefState.target = -1;
      reefState.locked = false;
    }
  }

  /**
   * Runs logic at the beginning of the periodic cycle.
   *
   * <p>Use this to poll external inputs, e.g. operator or vision, before the command scheduler
   * runs.
   */
  public void enterPeriodic() {
    pollReefState();

    scoringStrategy = scoringStrategyChooser.getSelected();

    // if the reef target is not locked by the operator use the scoring strategy
    // to automatically pick the most appropriate target based on the current state
    if (!reefState.locked && DriverStation.isTeleopEnabled()) {
      final var effectiveScoringStrategy =
          scoringStrategy == ScoringStrategy.AUTO
              ? DriverStation.getMatchType() == MatchType.Elimination
                  ? ScoringStrategy.SCORE
                  : ScoringStrategy.RP
              : scoringStrategy;

      // fill levels starting from the top (zero-based indexing here, i.e. L1 = 0, L2 = 1, etc.)
      var targetLevel = -1;

      if (effectiveScoringStrategy == ScoringStrategy.RP) {
        if (reefState.coralsL4.cardinality() < RP_CORALS_PER_LEVEL) {
          targetLevel = 3;
        } else if (reefState.coralsL3.cardinality() < RP_CORALS_PER_LEVEL) {
          targetLevel = 2;
        } else if (reefState.coralsL2.cardinality() < RP_CORALS_PER_LEVEL) {
          targetLevel = 1;
        }
      }

      // if target level is not set with RP strategy above, it means that we think
      // RP has been achieved, so we should focus on scoring
      if (effectiveScoringStrategy == ScoringStrategy.SCORE || targetLevel < 0) {
        if (reefState.coralsL4.cardinality() < CORALS_PER_LEVEL) {
          targetLevel = 3;
        } else if (reefState.coralsL3.cardinality() < CORALS_PER_LEVEL) {
          targetLevel = 2;
        } else if (reefState.coralsL2.cardinality() < CORALS_PER_LEVEL) {
          targetLevel = 1;
        } else {
          targetLevel = 0;
        }
      }

      // scoring at L1 is manual, nothing to do here
      if (targetLevel == 0) {
        reefState.target = -1;
      } else {
        // TODO: find the closest scoring position based on the robot location, other factors
        final var corals = reefState.getCorals(targetLevel);
        reefState.target = (byte) (targetLevel * 12 + corals.nextClearBit(0));
      }
    }
  }

  /**
   * Runs logic at the end of the periodic cycle.
   *
   * <p>Use this to synchronize the state with external modules, after the command scheduler runs.
   */
  public void exitPeriodic() {
    reefStateRobotPublisher.set(reefState);
  }

  /** Runs logic at the beginning of the autonomous cycle. */
  public void enterAutonomous() {
    reefState.reset();
  }
}
