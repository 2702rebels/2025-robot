package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {
  /** NT prefix for tunable keys */
  public static final String TunablePrefix = "/Tunable";

  /** Robot main loop cycle period in seconds */
  public static final double cyclePeriodSeconds = 0.02;

  public static final class DriveConstants {
    /** Maximum linear speed */
    public static final LinearVelocity maxSpeed = TunerConstants.kSpeedAt12Volts;

    /** Maximum linear acceleration */
    public static final LinearAcceleration maxAcceleration = maxSpeed.div(Seconds.of(0.5));

    /** Maximum angular velocity */
    public static final AngularVelocity maxAngularRate = RotationsPerSecond.of(1);

    /** Maximum angular acceleration */
    public static final AngularAcceleration maxAngularAcceleration =
        maxAngularRate.div(Seconds.of(0.5));

    /** PID configuration to pose chasing controllers */
    public static final class PoseChasing {
      /** kP used for the translation PID drive controllers */
      public static final double translationKp = 5;

      /** kI used for the translation PID drive controllers */
      public static final double translationKi = 0;

      /** kD used for the translation PID drive controllers */
      public static final double translationKd = 0;

      /** kP used for the rotation PID drive controllers */
      public static final double rotationKp = 3;

      /** kI used for the rotation PID drive controllers */
      public static final double rotationKi = 0;

      /** kD used for the rotation PID drive controllers */
      public static final double rotationKd = 0;
    }

    /** PID configuration for path following controllers */
    public static final class PathFollowing {
      /** kP used for the translation PID drive controllers */
      public static final double translationKp = 10;

      /** kI used for the translation PID drive controllers */
      public static final double translationKi = 0;

      /** kD used for the translation PID drive controllers */
      public static final double translationKd = 0;

      /** kP used for the rotation PID drive controllers */
      public static final double rotationKp = 7;

      /** kI used for the rotation PID drive controllers */
      public static final double rotationKi = 0;

      /** kD used for the rotation PID drive controllers */
      public static final double rotationKd = 0;

      /** PP PID constants for translation controllers */
      public static final PIDConstants translationPID =
          new PIDConstants(translationKp, translationKi, translationKd);

      /** PP PID constants for rotation controllers */
      public static final PIDConstants rotationPID =
          new PIDConstants(rotationKp, rotationKi, rotationKd);
    }

    /** PP robot configuration */
    public static final RobotConfig robotConfig = null;
  }
}
