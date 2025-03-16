package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedSparkMax;
import frc.robot.lib.Tunable;

public final class Climb extends SubsystemBase {
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 0.15;
  private static final double __kD = 0;

  private static final double conversionFactor =
      Math.PI * 3 / 50; // 3" diameter, 50:1 reduction ratio

  @Logged(name = "Winch", importance = Importance.INFO)
  private final LoggedSparkMax motor;

  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  // tolerance (inches)
  private static final double tolerance = 0.5;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  public Climb(int canID) {
    motor =
        new LoggedSparkMax(
            canID,
            (config) -> {
              config
                  .smartCurrentLimit(60)
                  .voltageCompensation(12)
                  .idleMode(IdleMode.kBrake)
                  .inverted(true);

              config.signals.primaryEncoderPositionAlwaysOn(true);

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(__kP)
                  .d(__kD);

              config.encoder.positionConversionFactor(conversionFactor);
            });

    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Climb", motor, ClosedLoopSlot.kSlot0);

    reset();
  }

  @Override
  public void periodic() {
    controller.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void reset() {
    encoder.setPosition(0);
  }

  public void setPosition(double value) {
    setpoint = value;
  }

  public boolean isAtPosition(double value) {
    return isAtPosition(value, tolerance);
  }

  public boolean isAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getPosition(), tolerance);
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return encoder.getPosition();
  }
}
