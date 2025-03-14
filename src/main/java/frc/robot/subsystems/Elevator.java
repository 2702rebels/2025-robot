package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTalonFXS;
import frc.robot.lib.PhoenixUtil;
import frc.robot.lib.Tunable;

public class Elevator extends SubsystemBase {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP = 1.95;
  private static final double __kD = 0;

  private static final double __kG0 = 0;
  private static final double __kG1 = 0.3;
  private static final double __kV = 0.075;
  private static final double __kMaxVelocity = 0; // rotations/sec
  private static final double __kMaxAcceleration = 300; // rotations/sec.sq
  private static final double __kJerk = 0;
  private static final double __kVExpo = 0.03;
  private static final double __kAExpo = 0.04;

  // conversion factor between rotation and linear movement
  // 3" wheel diameter, 8:1 reduction ratio
  private static final double conversionFactor = Math.PI * 3 / 8;

  // maximum safe travel
  private static final double maxTravel = 66;

  // tolerance (inches)
  private static final double tolerance = 1;

  @Logged(name = "Setpoint", importance = Importance.INFO)
  private double setpoint;

  @Logged(name = "Leader", importance = Importance.INFO)
  private final LoggedTalonFXS motorLeader;

  @Logged(name = "Follower", importance = Importance.INFO)
  private final LoggedTalonFXS motorFollower;

  private final Follower followerControl;
  private final MotionMagicExpoVoltage request = new MotionMagicExpoVoltage(0).withEnableFOC(false);

  public Elevator(int leaderCanID, int followerCanID, CANBus canbus) {
    motorLeader =
        new LoggedTalonFXS(
            leaderCanID,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.ExternalFeedback.ExternalFeedbackSensorSource =
                  ExternalFeedbackSensorSourceValue.Commutation;

              config.Slot0.kP = __kP;
              config.Slot0.kD = __kD;
              config.Slot0.kV = __kV;
              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
              config.MotionMagic.MotionMagicJerk = __kJerk;
              config.MotionMagic.MotionMagicExpo_kV = __kVExpo;
              config.MotionMagic.MotionMagicExpo_kA = __kAExpo;
            });

    motorFollower =
        new LoggedTalonFXS(
            followerCanID,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
              config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
              config.ExternalFeedback.ExternalFeedbackSensorSource =
                  ExternalFeedbackSensorSourceValue.Commutation;
            });

    // follow leader's output (non-inverted)
    followerControl = new Follower(motorLeader.getDeviceID(), false);

    PhoenixUtil.run(
        "Elevator Follow Leader",
        motorFollower.getDeviceID(),
        () -> motorFollower.setControl(followerControl));

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Elevator", motorLeader);
    Tunable.bindMotionProfile("Elevator/Motion", motorLeader);

    reset();
  }

  @Override
  public void periodic() {
    // elevator's effective weight changes as it extends higher, since
    // more sections need to be supported, this requires different kG
    // values to be applied when elevator is near ground vs. extended
    //
    // kG0 - applied when elevator is near zero
    // kG1 - applied otherwise

    final var gravityFF = getPosition() < 16 ? kG0.get() : kG1.get(); // kG0 below 16"
    motorLeader.setControl(
        request.withPosition(setpoint / conversionFactor).withFeedForward(gravityFF));
  }

  public void reset() {
    motorLeader.setPosition(0);
  }

  public void setPosition(double value) {
    setpoint = MathUtil.clamp(value, 0, maxTravel);
  }

  public boolean isAtPosition(double value) {
    return isAtPosition(value, tolerance);
  }

  public boolean isAtPosition(double value, double tolerance) {
    return MathUtil.isNear(value, getPosition(), tolerance);
  }

  public boolean isAbovePosition(double value) {
    return getPosition() >= value;
  }

  public boolean isBelowPosition(double value) {
    return getPosition() <= value;
  }

  @Logged(name = "Position", importance = Importance.INFO)
  public double getPosition() {
    return motorLeader.getPositionRotations() * conversionFactor;
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocity() {
    return motorLeader.getVelocityRPM() * conversionFactor / 60;
  }

  // spotless:off
  private Tunable.DoubleValue kG0 = Tunable.value("Elevator/FF/kG0", __kG0);
  private Tunable.DoubleValue kG1 = Tunable.value("Elevator/FF/kG1", __kG1);
  // spotless:on
}
