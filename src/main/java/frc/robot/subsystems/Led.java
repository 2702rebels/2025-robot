package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultCommand;
import java.util.Map;

public class Led extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  private final AddressableLEDBufferView[] segments =
      new AddressableLEDBufferView[Constants.LedConstants.kSegments.length];

  private static final LEDPattern kMask =
      LEDPattern.steps(Map.of(0.0, Color.kWhite, 0.2, Color.kBlack))
          .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(350));

  // these are the patterns that will be pushed to the buffer
  private LEDPattern patternTracking;
  private LEDPattern pattern;

  private boolean isTrackingSegment(int index) {
    return index == 0 || index == (segments.length / 2) || index == segments.length - 1;
  }

  public Led(int port) {
    leds = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(Constants.LedConstants.kLength);
    leds.setLength(buffer.getLength());

    // LED strip is arranged as follows in the addressable views
    // that are created below, indexed from the right bottom:
    //
    // mid left tracking   right
    // |           |           |
    // ****** ... *** ... ******
    // * -    right/left     - *
    // *                       *
    // *                       *
    // * -   side tracking   - *
    // * -                   - 0

    var offset = 0;
    for (var i = 0; i < segments.length; ++i) {
      final var length = Constants.LedConstants.kSegments[i];
      final var segment = buffer.createView(offset, offset + length - 1);

      segments[i] = i >= segments.length / 2 ? segment.reversed() : segment;
      offset += length;
    }

    // ensuring that the system is created with an off state automatically running
    pattern = LEDPattern.kOff;
    patternTracking = LEDPattern.kOff;

    leds.setData(buffer);
    leds.start();
  }

  private static LEDPattern breathe(Color color, double time) {
    return LEDPattern.solid(color).breathe(Units.Seconds.of(time));
  }

  private static LEDPattern blink(Color color) {
    return LEDPattern.solid(color).blink(Units.Second.of(0.15));
  }

  private static LEDPattern sweep(Color color) {
    return LEDPattern.solid(color).mask(kMask);
  }

  private static LEDPattern wave(Color color) {
    return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, color)
        .scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(100))
        .reversed();
  }

  /** Returns color based on the current alliance to use as the default in various patterns. */
  private static Color getDefaultColor() {
    final var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
        ? Color.kRed
        : Color.kBlue;
  }

  private Command runDefault(Runnable toRun) {
    return new DefaultCommand(toRun, this);
  }

  /** Commands LED strip to turn off. */
  public Command turnOff() {
    return runDefault(() -> pattern = LEDPattern.kOff);
  }

  /** Commands LED strip to a default state (based on alliance color). */
  public Command setDefault() {
    return runDefault(
        () -> {
          pattern =
              DriverStation.isDisabled()
                  ? breathe(Color.kPurple, 5)
                  : LEDPattern.solid(getDefaultColor());
        });
  }

  /** Commands LED strip to indicate that we see tags. */
  public Command tagsVisible() {
    return runDefault(() -> patternTracking = LEDPattern.solid(Color.kGreen));
  }

  /** Commands LED strip to indicate that we don't see tags. */
  public Command tagsNotVisible() {
    return runDefault(() -> patternTracking = LEDPattern.solid(Color.kDarkRed));
  }

  /** Commands LED strip to indicate scoring coral state. */
  public Command setScoringCoral() {
    return runDefault(() -> pattern = breathe(Color.kWhite, 1));
  }

  /** Commands LED strip to indicate scoring algae state. */
  public Command setScoringAlgae() {
    return runDefault(() -> pattern = breathe(Color.kSpringGreen, 1));
  }

  /** Commands LED strip to indicate intaking coral state. */
  public Command setIntakingCoral() {
    return runDefault(() -> pattern = sweep(Color.kWhite));
  }

  /** Commands LED strip to indicate intaking algae state. */
  public Command setIntakingAlgae() {
    return runDefault(() -> pattern = sweep(Color.kSpringGreen));
  }

  /** Commands LED strip to indicate holding coral state. */
  public Command setHoldingCoral() {
    return runDefault(() -> pattern = LEDPattern.solid(Color.kWhite));
  }

  /** Commands LED strip to indicate holding algae state. */
  public Command setHoldingAlgae() {
    return runDefault(() -> pattern = LEDPattern.solid(Color.kSpringGreen));
  }

  /** Commands LED strip to a wave pattern indicating climbing sequence. */
  public Command setClimbing(Led led) {
    return runDefault(() -> pattern = wave(getDefaultColor()));
  }

  /** Commands LED strip to indicate an action completed successfully. */
  public Command signalSuccess() {
    return startEnd(() -> pattern = blink(Color.kYellow), () -> turnOff()).withTimeout(0.8);
  }

  @Override
  public void periodic() {
    for (var i = 0; i < segments.length; ++i) {
      if (isTrackingSegment(i)) {
        patternTracking.applyTo(segments[i]);
      } else {
        pattern.applyTo(segments[i]);
      }
    }

    leds.setData(buffer);
  }
}
