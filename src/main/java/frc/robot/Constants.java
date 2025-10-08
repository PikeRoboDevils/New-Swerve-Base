package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public enum RobotState {
    L1Coral(0, 0),
    L2Coral(0, 0),
    L3Coral(0, 0),
    L4Coral(25.5, 0),
    Source(0, 0),
    L2Algae(0, 0),
    L3Algae(0, 0);

    public double WristPos;
    public double ElevatorPos;

    RobotState(double elevatorPos, double wristPos) {
      WristPos = wristPos;
      ElevatorPos = elevatorPos;
    }
  };

  public class gearRatios {

    public static double Arm = (11.0 / 42.0) * (1.0 / 25.0); // inverted
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // DRIVER PRACTICE
    public static final boolean driverPractice =
        false; // advantage scope has peformance issues when running mutiple robots change rendering
    // mode to standard or low
    public static final double ElevMinVolt = -12;
    public static final double ElevMaxVolt = 12;
  }

  // For Easier camera setup to be used with already made vision examples
  // Camera initiation can be found on line 376 of Vision swerve
  public static class PoseCameraConstants {
    // Cam 1 is roughly on top of the lower mount for elevator lokking in towards the reef tags.
    public static String CAM1N = "LEFT_CAM";

    public static Rotation3d CAM1R =
        new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(40));

    public static Translation3d CAM1T =
        new Translation3d(
            Units.inchesToMeters(-4), // transform of camera (dont forget forward+ left+ up+)
            Units.inchesToMeters(13),
            Units.inchesToMeters(8));

    // public static String CAM3N = "ELEV_CAM";

    // public static Rotation3d CAM3R =
    //     new Rotation3d(Math.PI, 0, 0);

    // public static Translation3d CAM3T =
    //     new Translation3d(
    //         Units.inchesToMeters(4.5), // transform of camera (dont forget forward+ left+ up+)
    //         Units.inchesToMeters(14), // to right
    //         Units.inchesToMeters(14));

    public static String CAM2N = "RIGHT_CAM";

    public static Rotation3d CAM2R =
        new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(180));

    public static Translation3d CAM2T =
        new Translation3d(
            Units.inchesToMeters(-4), // transform of camera (dont forget forward+ left+ up+)
            Units.inchesToMeters(-13),
            Units.inchesToMeters(8));

    // }

    // it goes up to 4 but it is commented out in SwerveVision
    public static final double maxVisionStdDevsDistance = 30;

    // it goes up to 4 but it is commented out in SwerveVision
  }

  public static final Mode currentMode = Mode.REAL; // TODO:IS MODE SET CURRECTLY??

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Encoders {

    public static final int WristChannel = 8;
    // public static final int WristChannelB = 0;
    public static final boolean WristReverse = false;

    public static final double kP_Wrist = 0.15;
    public static final double kI_Wrist = 0;
    public static final double kD_Wrist = 0;

    public static final int ElevatorChannelA = 2;
    public static final int ElevatorChannelB = 3;
    public static final boolean ElevatorReverse = false;

    // OLD ELEVATOR TUNING
    // public static final double kP_Elev = 7;
    // public static final double kI_Elev = 0;
    // public static final double kD_Elev = 0.0007;

    // public static final double kG_Elev = 0.090;
    // public static final double kV_Elev = 0.2;
    // public static final double maxVelocityElevator = 15;
    // public static final double maxAccelerationElevator = 20;
    // public static final double kS_Elev = 0;

    // NEW ELEVATOR TUNING
    public static final double kP_Elev = 5;
    public static final double kI_Elev = 0;
    public static final double kD_Elev = 0.05;

    public static final double kG_Elev = 0.3;
    public static final double kV_Elev = 0.6;
    public static final double maxVelocityElevator = 35; // THESE
    public static final double maxAccelerationElevator = 40; // NEXT
    public static final double kS_Elev = 0.2;
    ;
  }
}
