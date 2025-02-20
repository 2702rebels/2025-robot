package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final NetworkTable reefStateTable = nt.getTable("Robot/State/Reef");
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

  public static enum Intent {
    NONE,

    /** Intake coral form the floor */
    CORAL_FLOOR_INTAKE,

    /** Intake coral from the loading station */
    CORAL_STATION_INTAKE,

    /** Eject coral */
    CORAL_EJECT,

    /** Score coral at L1 */
    CORAL_SCORE_L1,

    /** Score coral at L2, L3, L4 per reef scoring strategy */
    CORAL_SCORE_AUTO,
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
  @Logged(name = "ScoringStrategy", importance = Importance.INFO)
  public ScoringStrategy scoringStrategy = ScoringStrategy.AUTO;

  /** Robot intent */
  @Logged(name = "Intent", importance = Importance.INFO)
  public Intent intent = Intent.NONE;

  /** Reef state (robot side) */
  public final ReefState reefState;

  // spotless:off
  @Logged(name = "Idle", importance = Importance.INFO)
  public final Trigger idle = new Trigger(() -> intent == Intent.NONE);

  @Logged(name = "ShouldIntakeCoralFloor", importance = Importance.INFO)
  public final Trigger shouldIntakeCoralFloor = new Trigger(() -> intent == Intent.CORAL_FLOOR_INTAKE);

  @Logged(name = "ShouldIntakeCoralStation", importance = Importance.INFO)
  public final Trigger shouldIntakeCoralStation = new Trigger(() -> intent == Intent.CORAL_STATION_INTAKE);

  @Logged(name = "ShouldIntakeCoral", importance = Importance.INFO)
  public final Trigger shouldIntakeCoral = shouldIntakeCoralFloor.or(shouldIntakeCoralStation);

  @Logged(name = "ShouldEjectCoral", importance = Importance.INFO)
  public final Trigger shouldEjectCoral = new Trigger(() -> intent == Intent.CORAL_EJECT);

  @Logged(name = "ShouldScoreCoralL1", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL1 = new Trigger(() -> intent == Intent.CORAL_SCORE_L1);

  @Logged(name = "ShouldScoreCoralAuto", importance = Importance.INFO)
  public final Trigger shouldScoreCoralAuto = new Trigger(() -> intent == Intent.CORAL_SCORE_AUTO);

@Logged(name = "ShouldScoreCoral", importance = Importance.INFO)
  public final Trigger shouldScoreCoral = shouldScoreCoralL1.or(shouldScoreCoralAuto);
  // spotless:on

  /** Gets the score level for automatic scoring (1..4) */
  public int getScoreLevel() {
    return reefState.target > 0 ? (reefState.target / 12) + 1 : -1;
  }

  public Command setIntent(Intent intent) {
    return Commands.runOnce(
            () -> {
              this.intent = intent;
            })
        .withName("SetIntent");
  }

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
