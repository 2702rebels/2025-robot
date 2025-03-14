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
  private static final double __kArmP = 0.05;
  private static final double __kArmD = 0;
  private static final double __kWinchP = 0.15;
  private static final double __kWinchD = 0;

  private static final double conversionFactorArm = 360 / 100; // to degrees, 100:1 reduction ratio
  private static final double conversionFactorWinch =
      Math.PI * 3 / 50; // 3" diameter, 50:1 reduction ratio

  @Logged(name = "Arm", importance = Importance.INFO)
  private final LoggedSparkMax motorArm;

  @Logged(name = "Winch", importance = Importance.INFO)
  private final LoggedSparkMax motorWinch;

  private final RelativeEncoder encoderArm;
  private final RelativeEncoder encoderWinch;
  private final SparkClosedLoopController controllerArm;
  private final SparkClosedLoopController controllerWinch;

  // tolerance arm (degrees)
  private static final double toleranceArm = 3;

  // tolerance winch (inches)
  private static final double toleranceWinch = 0.5;

  @Logged(name = "Arm/Setpoint", importance = Importance.INFO)
  private double setpointArm;

  @Logged(name = "Winch/Setpoint", importance = Importance.INFO)
  private double setpointWinch;

  public Climb(int armCanID, int winchCanID) {
    motorArm =
        new LoggedSparkMax(
            armCanID,
            (config) -> {
              config.smartCurrentLimit(30).voltageCompensation(12).idleMode(IdleMode.kBrake);

              config.signals.primaryEncoderPositionAlwaysOn(true);

              config
                  .closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .velocityFF(0)
                  .p(__kArmP)
                  .d(__kArmD);

              config.encoder.positionConversionFactor(conversionFactorArm);
            });

    motorWinch =
        new LoggedSparkMax(
            winchCanID,
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
                  .p(__kWinchP)
                  .d(__kWinchD);

              config.encoder.positionConversionFactor(conversionFactorWinch);
            });

    encoderArm = motorArm.getEncoder();
    encoderWinch = motorWinch.getEncoder();
    controllerArm = motorArm.getClosedLoopController();
    controllerWinch = motorWinch.getClosedLoopController();

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Climb/Arm", motorArm, ClosedLoopSlot.kSlot0);
    Tunable.bind("Climb/Winch", motorWinch, ClosedLoopSlot.kSlot0);

    reset();
  }

  @Override
  public void periodic() {
    controllerArm.setReference(setpointArm, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    controllerWinch.setReference(setpointWinch, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void reset() {
    encoderArm.setPosition(0);
    encoderWinch.setPosition(0);
  }

  public void setArmPosition(double value) {
    setpointArm = value;
  }

  public boolean isArmAtPosition(double value) {
    return isArmAtPosition(value, toleranceArm);
  }

  public boolean isArmAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getArmPosition(), tolerance);
  }

  @Logged(name = "Arm/Position", importance = Importance.INFO)
  public double getArmPosition() {
    return encoderArm.getPosition();
  }

  public void setWinchPosition(double value) {
    setpointWinch = value;
  }

  public boolean isWinchAtPosition(double value) {
    return isWinchAtPosition(value, toleranceWinch);
  }

  public boolean isWinchAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getWinchPosition(), tolerance);
  }

  @Logged(name = "Winch/Position", importance = Importance.INFO)
  public double getWinchPosition() {
    return encoderWinch.getPosition();
  }
}
