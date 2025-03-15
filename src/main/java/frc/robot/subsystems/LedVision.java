package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.commands.DefaultCommand;
import java.util.Map;
import java.util.function.Supplier;

public class LedVision extends SubsystemBase {
  private final LedResource resource;
  private final Supplier<RobotState.LocalizationState> stateSupplier;

  public LedVision(LedResource resource, Supplier<RobotState.LocalizationState> stateSupplier) {
    this.resource = resource;
    this.stateSupplier = stateSupplier;

    // set default command
    setDefaultCommand(createDefault());
  }

  // "evergreen" patterns
  private static LEDPattern assistPattern = LedResource.blink(Color.kGreen);

  // localization-state-specific patterns
  private static final Map<RobotState.LocalizationState, LEDPattern> kStatePatterns =
      Map.of(
          RobotState.LocalizationState.NONE, LEDPattern.kOff,
          RobotState.LocalizationState.TRUSTED, LEDPattern.solid(Color.kGreen),
          RobotState.LocalizationState.UNTRUSTED, LEDPattern.solid(Color.kDarkRed));

  private Command runDefault(Runnable toRun) {
    return new DefaultCommand(toRun, this);
  }

  /** Creates default command for this LED subsystem that is state-driven */
  public Command createDefault() {
    return run(() -> {
          resource.setVisionPattern(kStatePatterns.get(stateSupplier.get()));
        })
        .ignoringDisable(true)
        .withName("LedVisionDefault");
  }

  /** Commands LED strip to indicate vision-assisted driving / sequence. */
  public Command signalAssist() {
    return runDefault(() -> resource.setVisionPattern(assistPattern));
  }
}
