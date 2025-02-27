package frc.robot.lib;

/** A value with attached timestamp in microseconds. */
public record TimestampedValue<T>(long timestamp, T value) {}
