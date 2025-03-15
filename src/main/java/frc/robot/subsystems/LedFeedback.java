package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.commands.DefaultCommand;
import java.util.Map;
import java.util.function.Supplier;

public class LedFeedback extends SubsystemBase {
  private final LedResource resource;
  private final Supplier<RobotState.GameState> stateSupplier;

  public LedFeedback(LedResource resource, Supplier<RobotState.GameState> stateSupplier) {
    this.resource = resource;
    this.stateSupplier = stateSupplier;

    // set default command
    setDefaultCommand(createDefault());
  }

  // "evergreen" patterns
  private static LEDPattern defaultPatternRed = LEDPattern.solid(Color.kRed);
  private static LEDPattern defaultPatternBlue = LEDPattern.solid(Color.kBlue);
  private static LEDPattern disabledPattern = LedResource.breathe(Color.kPurple, 5);
  private static LEDPattern estoppedPattern = LedResource.blink(Color.kPurple);
  private static LEDPattern successPattern = LedResource.blink(Color.kYellow);
  private static LEDPattern readyPattern = LedResource.blink(Color.kWhite);

  // game-state-specific patterns
  private static final Map<RobotState.GameState, LEDPattern> kStatePatterns =
      Map.ofEntries(
          Map.entry(RobotState.GameState.SCORING_CORAL, LedResource.breathe(Color.kWhite, 3)),
          Map.entry(RobotState.GameState.SCORING_ALGAE, LedResource.breathe(Color.kSpringGreen, 3)),
          Map.entry(RobotState.GameState.INTAKING_CORAL, LedResource.sweep(Color.kWhite)),
          Map.entry(RobotState.GameState.INTAKING_ALGAE, LedResource.sweep(Color.kSpringGreen)),
          Map.entry(RobotState.GameState.HOLDING_CORAL, LEDPattern.solid(Color.kWhite)),
          Map.entry(RobotState.GameState.HOLDING_ALGAE, LEDPattern.solid(Color.kSpringGreen)),
          Map.entry(RobotState.GameState.CLIMBING, LedResource.wave(Color.kYellow)));

  private Command runDefault(Runnable toRun) {
    return new DefaultCommand(toRun, this);
  }

  /** Creates default command for this LED subsystem that is state-driven */
  public Command createDefault() {
    return run(() -> {
          var pattern = LEDPattern.kOff;

          // special handling of estopped/disabled/default states
          if (DriverStation.isEStopped()) {
            pattern = estoppedPattern;
          } else if (DriverStation.isDisabled()) {
            pattern = disabledPattern;
          } else {
            final var state = stateSupplier.get(); // poll current state
            if (state == RobotState.GameState.IDLE) {
              // select pattern based on the alliance color
              final var alliance = DriverStation.getAlliance();
              pattern =
                  alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
                      ? defaultPatternRed
                      : defaultPatternBlue;
            } else {
              pattern = kStatePatterns.get(state);
            }
          }

          resource.setFeedbackPattern(pattern);
        })
        .ignoringDisable(true)
        .withName("LedFeedbackDefault");
  }

  /** Commands LED strip to indicate an action completed successfully. */
  public Command signalSuccess() {
    return runDefault(() -> resource.setFeedbackPattern(successPattern)).withTimeout(0.8);
  }

  /** Commands LED strip to indicate a ready state. */
  public Command signalReady() {
    return runDefault(() -> resource.setFeedbackPattern(readyPattern));
  }
}
