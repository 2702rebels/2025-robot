package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.led.Led;

// this class houses (mostly) InstantCommands to control the LED light strips on the robot :)
public class LedCommands {

    // to control the top ends of each LED strip, indicating whether two AprilTags are seen or not (green if seen, dim red if not)
    // pass in the Led subystem and a lambda to the boolean that stores if AprilTags are seen
    public static InstantCommand odometryUpdated(Led led) {
        return new InstantCommand(() -> led.trackingColor(Color.kGreen));
    }

    public static InstantCommand odometryNotUpdated(Led led) {
        return new InstantCommand(() -> led.trackingColor(Color.kDarkRed));
    }

    // this command flashes the light strips yellow 3x, when a beam break or limit switch is tripped or once a scoring sequence has completed
    // pass in the LED subsystem and a lambda to the alliance color, ensuring the lights return to the off state after flashing
    public static Command completedAction(Led led, Supplier<Color> m_allianceColor) {
        return new StartEndCommand(
            () -> led.stateBlink(Color.kYellow),  // Start action
            () -> led.turnOff(), led
            ).withTimeout(0.8);
    }

    // simple command to shut off LED strips (not the tag tracking portion, modify subsystem to achieve this)
    // pass in the Led subsystem
    public static InstantCommand turnOff(Led led) {
        // System.out.println("LIGHTS OFF!");
        return new InstantCommand(() -> led.turnOff());
    }

    // breathing state for the robot lights when disabled
    // pass in the Led subsystem
    public static InstantCommand disabled(Led led) {
        // System.out.println("DISABLED!");
        return new InstantCommand(() -> led.breathe(Color.kRed, 5), led);
    }

    public static InstantCommand scoringCoral(Led led) {
        // System.out.println("DISABLED!");
        return new InstantCommand(() -> led.breathe(Color.kWhite, 1), led);
    }

    public static InstantCommand scoringAlgae(Led led) {
        // System.out.println("DISABLED!");
        return new InstantCommand(() -> led.breathe(Color.kSpringGreen, 1), led);
    }

    // in teleop, this keeps the robot lights the alliance color when not doing a major task
    // pass in the Led subsystem and a lambda to the alliance color from RobotContainer
    public static InstantCommand allianceColor(Led led, Supplier<Color> color) {
        // System.out.println("TELEOP");
        return new InstantCommand(() -> led.solid(color.get()), led);
    }

    // when trying to intake Coral, the lights stay a solid white
    // pass in the Led subsystem
    public static InstantCommand intakingCoral(Led led) {
        // System.out.println("TELEOP");
        return new InstantCommand(() -> led.sweepLights(Color.kWhite), led);
    }

    // when trying to manipulate or intake an Algae, the lights stay a solid cyan
    // pass in the Led subsystem
    public static InstantCommand intakingAlgae(Led led) {
        // System.out.println("TELEOP");
        return new InstantCommand(() -> led.sweepLights(Color.kSpringGreen), led);
    }

    // static lights for when robot is in "coral" mode
    public static InstantCommand holdingCoral(Led led) {
        // solid colour state for when a coral is held
        return new InstantCommand (() -> led.solid(Color.kWhite), led);
    }

    // static lights for when robot is in "algae" mode
    public static InstantCommand holdingAlgae(Led led) {
        // solid colour state for when a coral is held
        return new InstantCommand (() -> led.solid(Color.kSpringGreen), led);
    }

    // wave pattern for when the robot enters the climbing sequence, matching alliance color
    // pass in the Led subsystem and a lambda to the alliance color from RobotContainer
    public static InstantCommand climbing(Led led, Supplier<Color> m_allianceColor) {
        // System.out.println("DISABLED!");
        return new InstantCommand(() -> led.climbingAnimation(m_allianceColor.get()), led);
    }
}