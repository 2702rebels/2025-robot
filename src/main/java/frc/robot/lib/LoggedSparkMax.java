package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import java.util.function.Consumer;

public class LoggedSparkMax extends SparkMax {
  public LoggedSparkMax(int deviceId) {
    this(deviceId, null);
  }

  public LoggedSparkMax(int deviceId, Consumer<SparkMaxConfig> configurator) {
    super(deviceId, MotorType.kBrushless);

    final var config = new SparkMaxConfig();
    configure(config);

    // apply external configuration if specified
    if (configurator != null) {
      configurator.accept(config);
    }

    RevUtil.config(this, config);
  }

  protected void configure(SparkMaxConfig config) {
    // limit switches: disable motor controls, use only for sensor purposes
    config.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
  }

  @Logged(name = "Temperature", importance = Importance.INFO)
  public double getTemperature() {
    return getMotorTemperature();
  }

  @Logged(name = "AppliedVolts", importance = Importance.INFO)
  public double getAppliedVolts() {
    return getAppliedOutput() * getBusVoltage();
  }

  @Logged(name = "CurrentAmps", importance = Importance.INFO)
  public double getCurrentAmps() {
    return getOutputCurrent();
  }
}
