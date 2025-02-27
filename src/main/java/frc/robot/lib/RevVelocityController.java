package frc.robot.lib;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Combines REV motor and SparkMax controller configured for a closed-loop velocity control mode.
 */
public class RevVelocityController extends LoggedSparkMax {
  public final SparkClosedLoopController controller;
  public final RelativeEncoder encoder;

  public RevVelocityController(
      int canID, String name, double gearRatio, double kP, double kI, double kD, double kF) {
    super(
        canID,
        (config) -> {
          config
              .smartCurrentLimit(50)
              .voltageCompensation(12)
              .idleMode(IdleMode.kCoast)
              .signals
              .primaryEncoderVelocityAlwaysOn(true);

          // conversion factor based on the gear ratio
          config
              .encoder
              .positionConversionFactor(1 / gearRatio)
              .velocityConversionFactor(1 / gearRatio);

          // closed-loop controller
          // use slot 1 for velocity control
          config
              .closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              // https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
              .velocityFF(kF, ClosedLoopSlot.kSlot1)
              .p(kP, ClosedLoopSlot.kSlot1)
              .i(kI, ClosedLoopSlot.kSlot1)
              .d(kD, ClosedLoopSlot.kSlot1);
        });

    encoder = getEncoder();
    controller = getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind(name, this, ClosedLoopSlot.kSlot1);
  }

  public void setVelocity(double value) {
    controller.setReference(value, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void stop() {
    stopMotor();
  }
}
