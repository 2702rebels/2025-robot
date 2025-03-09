package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.ReefState;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.Tunable;
import frc.robot.lib.VisionState;
import java.util.BitSet;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private static final int RP_CORALS_PER_LEVEL = 5;
  private static final int CORALS_PER_LEVEL = 12;

  // timestamped log of recently measured (within last second) chassis speeds
  private final TimestampedCircularBuffer<ChassisSpeeds> chassisSpeeds =
      new TimestampedCircularBuffer<>(50);

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  // external supplier of observed historical robot poses
  private Function<Long, Optional<Pose2d>> historicalPoseSupplier;

  // external vision state supplier
  private Supplier<VisionState> visionStateSupplier;

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

    /** Hover intake to allow trapped game pieces to escape */
    HOVER,

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

    /** Score coral at L4 (final motion) */
    CORAL_SCORE_L4_FINAL,

    /** Intake algae (L2 low) */
    ALGAE_INTAKE_L2,

    /** Intake algae (L3 high) */
    ALGAE_INTAKE_L3,

    /** Score algae into the net */
    ALGAE_SCORE_NET,

    /** Eject algae into the processor */
    ALGAE_EJECT,
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

  @Logged(name = "ShouldHover", importance = Importance.INFO)
  public final Trigger shouldHover = new Trigger(() -> intent == Intent.HOVER);

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

  @Logged(name = "ShouldScoreCoralL23", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL23 = new Trigger(() -> {
    if (intent == Intent.CORAL_SCORE_AUTO) {
      final var level = getScoreLevel();
      return level == 2 || level == 3;
    }
    return false;
  });

  @Logged(name = "ShouldScoreCoralL4", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL4 = new Trigger(() -> intent == Intent.CORAL_SCORE_AUTO && getScoreLevel() == 4);

  @Logged(name = "ShouldScoreCoralL4Final", importance = Importance.INFO)
  public final Trigger shoulScoreCoralL4Final = new Trigger(()-> intent == Intent.CORAL_SCORE_L4_FINAL);

  @Logged(name = "ShouldIntakeAlgaeL2", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgaeL2 = new Trigger(() -> intent == Intent.ALGAE_INTAKE_L2);

  @Logged(name = "ShouldIntakeAlgaeL3", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgaeL3 = new Trigger(() -> intent == Intent.ALGAE_INTAKE_L3);

  @Logged(name = "ShouldIntakeAlgae", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgae = shouldIntakeAlgaeL2.or(shouldIntakeAlgaeL3);

  @Logged(name = "ShouldEjectAlgae", importance = Importance.INFO)
  public final Trigger shouldEjectAlgae = new Trigger(() -> intent == Intent.ALGAE_EJECT);

  @Logged(name = "ShouldScoreAlgaeNet", importance = Importance.INFO)
  public final Trigger shouldScoreAlgaeNet = new Trigger(() -> intent == Intent.ALGAE_SCORE_NET);
  // spotless:on

  private VisionState visionState;

  @Logged(name = "VisionPose", importance = Importance.INFO)
  public Pose2d getVisionPose() {
    return visionState != null && visionState.pose.timestamp() > 0
        ? visionState.pose.value()
        : null;
  }

  @Logged(name = "VisionPoseAge", importance = Importance.DEBUG)
  public double getVisionPoseAge() {
    if (visionState != null && visionState.pose.timestamp() > 0) {
      return (RobotController.getFPGATime() - visionState.pose.timestamp()) / 1000000.0;
    }

    return -1;
  }

  @Logged(name = "ScoringPose", importance = Importance.DEBUG)
  public Pose2d getScoringPoseDebug() {
    return scoringPose.isPresent() ? scoringPose.get() : null;
  }

  /** Target pose to supply to driveToPose command */
  private Optional<Pose2d> targetPose = Optional.empty();

  /** Target pose for auto scoring */
  private Optional<Pose2d> scoringPose = Optional.empty();

  /** Most recent robot pose */
  private Optional<Pose2d> robotPose = Optional.empty();

  /** Trigger for scoring pose presence. */
  @Logged(name = "HasScoringPose", importance = Importance.INFO)
  public final Trigger hasScoringPose = new Trigger(() -> scoringPose.isPresent());

  /** Trigger for scoring readiness */
  @Logged(name = "ScoringReady", importance = Importance.INFO)
  public final Trigger scoringReady = new Trigger(() -> !visionScoring.get() || isAtScoringPose());

  /** Binds external historical pose supplier. */
  public void setHistoricalPoseSupplier(Function<Long, Optional<Pose2d>> value) {
    historicalPoseSupplier = value;
  }

  /** Binds external vision state supplier. */
  public void setVisionStateSupplier(Supplier<VisionState> value) {
    visionStateSupplier = value;
  }

  /** Gets the score level for automatic scoring (1..4) */
  public int getScoreLevel() {
    if (DriverStation.isAutonomous() && DriverStation.isAutonomousEnabled()) {
      return 2;
    }

    return reefState.target > 0 ? (reefState.target / 12) + 1 : -1;
  }

  public Command setIntent(Intent intent) {
    return Commands.runOnce(
            () -> {
              if (intent == Intent.CORAL_SCORE_AUTO && visionScoring.get()) {
                scoringPose =
                    robotPose.isPresent()
                        ? getClosestScoringPose(robotPose.get())
                        : Optional.empty();

                // do not set intent since we are trying to score via vision
                // and no target is available
                if (scoringPose.isEmpty()) {
                  return;
                }
              }

              // reset scoring pose when intent is removed
              if (intent != Intent.CORAL_SCORE_AUTO) {
                scoringPose = Optional.empty();
              }

              this.intent = intent;
            })
        .withName("SetIntent");
  }

  public void setTargetPose(Pose2d pose) {
    targetPose = Optional.of(pose);
  }

  public Optional<Pose2d> getTargetPose() {
    return targetPose;
  }

  public Optional<Pose2d> getScoringPose() {
    return scoringPose;
  }

  public boolean isAtScoringPose() {
    if (scoringPose.isPresent() && robotPose.isPresent()) {
      final var _scoringPose = scoringPose.get();
      final var _robotPose = robotPose.get();

      final var d = _scoringPose.getTranslation().getDistance(_robotPose.getTranslation());
      final var r =
          Math.abs(_scoringPose.getRotation().getDegrees() - _robotPose.getRotation().getDegrees());

      // THESE ARE HARDCODED RIGHT NOW
      return d < Units.inchesToMeters(2) && r < 5;
    }

    return false;
  }

  public Optional<VisionState> getVisionState() {
    return Optional.ofNullable(visionState);
  }

  public Command resetScoringPose() {
    return new InstantCommand(() -> scoringPose = Optional.empty());
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

    // poll vision state
    if (visionStateSupplier != null) {
      visionState = visionStateSupplier.get();
    }

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

  /** Gets the closest scoring pose based on the closes reef faces if applicable. */
  public Optional<Pose2d> getClosestScoringPose(Pose2d robotPose) {
    final var scoringLevel = getScoreLevel();
    final var scoringPoses =
        scoringLevel == 4 ? GameConstants.reefScoringPosesL4 : GameConstants.reefScoringPoses;

    final var robotXY = robotPose.getTranslation();

    // find two closest faces of the reef
    final var faceDistances = new double[6];
    for (var i = 0; i < FieldConstants.Reef.faces.length; ++i) {
      final var face = FieldConstants.fieldPose.apply(FieldConstants.Reef.faces[i]);
      faceDistances[i] = robotXY.getDistance(face.getTranslation());
    }

    int face1 = 0;
    int face2 = 1;

    if (faceDistances[0] > faceDistances[1]) {
      face1 = 1;
      face2 = 0;
    }

    for (var i = 2; i < faceDistances.length; ++i) {
      if (faceDistances[i] < faceDistances[face1]) {
        face2 = face1;
        face1 = i;
      } else if (faceDistances[i] < faceDistances[face2]) {
        face2 = i;
      }
    }

    // find the closest scoring position that is available
    Pose2d currentPose = null;
    double distance = -1;

    final var position1a = getScoringPosition(scoringLevel, face1 * 2, scoringPoses, robotXY);
    final var position1b = getScoringPosition(scoringLevel, face1 * 2 + 1, scoringPoses, robotXY);
    final var position2a = getScoringPosition(scoringLevel, face2 * 2, scoringPoses, robotXY);
    final var position2b = getScoringPosition(scoringLevel, face2 * 2 + 1, scoringPoses, robotXY);

    if (position1a.isPresent()) {
      currentPose = position1a.get().getFirst();
      distance = position1a.get().getSecond();
    }

    if (position1b.isPresent() && (distance < 0 || position1b.get().getSecond() < distance)) {
      currentPose = position1b.get().getFirst();
      distance = position1b.get().getSecond();
    }

    if (position2a.isPresent() && (distance < 0 || position2a.get().getSecond() < distance)) {
      currentPose = position2a.get().getFirst();
      distance = position2a.get().getSecond();
    }

    if (position2b.isPresent() && (distance < 0 || position2b.get().getSecond() < distance)) {
      currentPose = position2b.get().getFirst();
      distance = position2b.get().getSecond();
    }

    return Optional.ofNullable(currentPose);
  }

  private Optional<Pair<Pose2d, Double>> getScoringPosition(
      int level, int index, Pose2d[] poses, Translation2d robotXY) {
    if (reefState.isPositionScored(level, index)) {
      return Optional.empty();
    }

    final var pose = FieldConstants.fieldPose.apply(poses[index]);
    return Optional.of(Pair.of(pose, robotXY.getDistance(pose.getTranslation())));
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

  /** Updates odometry based on direct and vision observations. */
  public void updateOdometry(long timestamp, Pose2d pose, ChassisSpeeds fieldSpeeds) {
    robotPose = Optional.of(pose);
    chassisSpeeds.enqueue(timestamp, fieldSpeeds);
  }

  /** Returns observed chassis speeds at the specified timestamp. */
  public Optional<ChassisSpeeds> getSpeedsAt(long timestamp) {
    return chassisSpeeds.findAt(timestamp);
  }

  /** Returns observed pose at the specified timestamp. */
  public Optional<Pose2d> getPoseAt(long timestamp) {
    return historicalPoseSupplier.apply(timestamp);
  }

  public static final Tunable.BooleanValue visionScoring =
      Tunable.value("Control/VisionScoring", true);
}
