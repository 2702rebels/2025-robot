// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// this is the Led subsystem to control the signalling lights of the robot
public class Led extends SubsystemBase {

    // init local variables
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_bufferLeds;
    private final AddressableLEDBufferView m_right;
    private final AddressableLEDBufferView m_middleTrackingSegment;
    private final AddressableLEDBufferView m_left;
    private final AddressableLEDBufferView m_middleRight;
    private final AddressableLEDBufferView m_middleLeft;
    private final AddressableLEDBufferView m_rightTrackingSegment;
    private final AddressableLEDBufferView m_leftTrackingSegment;
    private final int port = LedConstants.PWM_PORT;
    private final int length = LedConstants.LENGTH;
    private final int sideLength = LedConstants.SIDE_STRIP_LENGTH;
    private final int middleHalf = (LedConstants.MIDDLE_STRIP_LENGTH / 2) - 2;
    private final int middleTrackingSegment = LedConstants.MIDDLE_TRACKING_SEGMENT_LENGTH;
    private final int sideTrackingSegment = LedConstants.SIDE_TRACKING_SEGMENT_LENGTH;

    // these are the patterns that will be pushed to the buffer
    private LEDPattern patternTracking;
    private LEDPattern pattern;

    // subsystem constructor takes no arguments, pulls from LedConstants.java file
    public Led() {

        // PWM port, buffer with length, set member var to be the length of the buffer
        m_leds = new AddressableLED(port);
        m_bufferLeds = new AddressableLEDBuffer(length);
        m_leds.setLength(m_bufferLeds.getLength());

        // creating a custom view for each part of the chain: 4 strips with 1 tag tracking segment each
        m_rightTrackingSegment = m_bufferLeds.createView(0, sideTrackingSegment - 1);
        m_right = m_bufferLeds.createView(sideTrackingSegment, (sideLength - 1));

        m_middleRight = m_bufferLeds.createView(sideLength, sideLength + middleHalf - 1);
        m_middleTrackingSegment = m_bufferLeds.createView(sideLength + middleHalf, sideLength + middleHalf + middleTrackingSegment - 1);
        m_middleLeft = m_bufferLeds.createView(sideLength + middleHalf + middleTrackingSegment, sideLength + (2*middleHalf) + middleTrackingSegment - 1).reversed();

        m_left = m_bufferLeds.createView(sideLength + (2*middleHalf) + middleTrackingSegment, length - sideTrackingSegment - 1).reversed();
        m_leftTrackingSegment = m_bufferLeds.createView(length - sideTrackingSegment, length - 1).reversed();

        // ensuring that the system is created with an off state automatically running
        pattern = LEDPattern.kOff;
        patternTracking = LEDPattern.kOff;

        m_leds.setData(m_bufferLeds);
        m_leds.start();
    }

    // ensures patternTracking is updatable
    public void trackingColor(Color color) {
        patternTracking = LEDPattern.solid(color);
    }

    // off method in case it's needed
    public void turnOff() {
        pattern = LEDPattern.kOff;
    }

    // blink method for meeting robot state criteria
    public void stateBlink(Color color) {
        pattern = LEDPattern.solid(color).blink(Units.Second.of(0.15));
    }

    // fast chaser-like sweeping light method for scoring
    public void sweepLights(Color color) {
        Map<Double, Color> maskInitial = Map.of(0.0, Color.kWhite, 0.2, Color.kBlack);

        pattern = LEDPattern.solid(color);
        LEDPattern mask = LEDPattern.steps(maskInitial).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(350));

        pattern = pattern.mask(mask);
    }

    // a wave-like pattern method
    public void climbingAnimation(Color color) {
        // Distance spacing = Units.Meter.of(1 / 120);
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, color).scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(100)).reversed();

    }

    // the standard breathing method for the disabled state
    public void breathe(Color color, double time) {
        pattern = LEDPattern.solid(color).breathe(Units.Seconds.of(time));
    }

    // sets a solid color to the main strip, unlike trackingColor
    public void solid(Color color) {
        pattern = LEDPattern.solid(color);
    }

    // every loop, apply the patterns to each bufferView and then update the strip using the buffer
    @Override
    public void periodic() {
        pattern.applyTo(m_left);
        pattern.applyTo(m_right);
        pattern.applyTo(m_middleRight);
        pattern.applyTo(m_middleLeft);

        patternTracking.applyTo(m_middleTrackingSegment);
        patternTracking.applyTo(m_rightTrackingSegment);
        patternTracking.applyTo(m_leftTrackingSegment);

        m_leds.setData(m_bufferLeds);
    }

}