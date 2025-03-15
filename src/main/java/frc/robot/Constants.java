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
  public static final class Settings {
    /** Logging */
    public static final boolean loggingEnabled = true;

    /** Logging timing stats */
    public static final boolean statsEnabled = true;

    /** SysId bindings */
    public static final boolean sysIdBindings = false;

    /** Driving disabled */
    public static final boolean drivingDisabled = false;
  }

  /** NT prefix for tunable keys */
  public static final String tunablePrefix = "/Tunable";

  /** Robot main loop cycle period in seconds */
  public static final double cyclePeriodSeconds = 0.02;

  /** Required pivot angle in the initial position (in degrees) */
  public static final double initialPivotAngle = 140;

  public static final class Network {
    public static final int UDP_PORT_OBJECT_DETECTOR = 27021;
    public static final int UDP_PORT_APRILTAG_DETECTOR = 27022;
  }

  /** RoboRIO CAN identifiers */
  public static final class CanID {
    public static final int MOTOR_PIVOT_LEADER = 13;
    public static final int MOTOR_PIVOT_FOLLOWER = 12;
    public static final int MOTOR_MANIPULATOR_CORAL = 14;
    public static final int MOTOR_MANIPULATOR_GUIDE = 15;
    public static final int MOTOR_CLIMB_WINCH = 16;
    public static final int MOTOR_CLIMB_ARM = 17;
  }

  /** CANivore identifiers */
  public static final class CanivoreID {
    public static final int MOTOR_ELEVATOR_LEADER = 15;
    public static final int MOTOR_ELEVATOR_FOLLOWER = 16;
  }

  public static final class RevConstants {
    /**
     * REV NEO550 motor constants
     * https://docs.revrobotics.com/brushless/neo/550#motor-specifications
     */
    public static final class NEO550 {
      public static final double kV = 917;
      public static final double freeSpeed = 11000;
    }

    /**
     * REV NEO v1 motor constants
     * https://docs.revrobotics.com/brushless/neo/v1.1/neo-v1#motor-specifications
     */
    public static final class NEOv1 {
      public static final double kV = 473;
      public static final double freeSpeed = 5676;
    }
  }

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
      public static final double translationKp = 2;

      /** kI used for the translation PID drive controllers */
      public static final double translationKi = 0;

      /** kD used for the translation PID drive controllers */
      public static final double translationKd = 0;

      /** kP used for the rotation PID drive controllers */
      public static final double rotationKp = 3.5;

      /** kI used for the rotation PID drive controllers */
      public static final double rotationKi = 0;

      /** kD used for the rotation PID drive controllers */
      public static final double rotationKd = 0;
    }

    /** PID configuration for path following controllers */
    public static final class PathFollowing {
      /** kP used for the translation PID drive controllers */
      public static final double translationKp = 0.4;

      /** kI used for the translation PID drive controllers */
      public static final double translationKi = 0;

      /** kD used for the translation PID drive controllers */
      public static final double translationKd = 0;

      /** kP used for the rotation PID drive controllers */
      public static final double rotationKp = 0.4;

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

  public static final class LedConstants {
    /** PWM port on RoboRio */
    public static final int PWM_PORT = 0;

    /* Total length of LED strip (number of addressable LEDs) */
    public static final int kLength = 39;

    /** Individual segments */
    public static final int[] kSegments = {2, 7, 7, 7, 7, 7, 2};
  }
}
