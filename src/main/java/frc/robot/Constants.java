package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
 
public final class Constants
{

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton
    {
        public static final double kp = 0.75;
        public static final double ki = 0.38;
        public static final double kd = 0;
        
        // P = 0.46 - I = 0.17 - D = 0.01 

        public static final double kpH = 0;
        public static final double kiH = 0;
        public static final double kdH = 0;

        // P = 0 - I = 0 - D = 0 


        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase
    {
public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants
    {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
        private static final RobotType robot = RobotType.ROBOT_2023C;
        public static final double loopPeriodSecs = 0.02;
        public static final boolean tuningMode = false;
      
        public static boolean invalidRobotAlertSent = false;
      
        public static RobotType getRobot() {
          if (!disableHAL && RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
              if (!invalidRobotAlertSent) {
                new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                    .set(true);
                invalidRobotAlertSent = true;
              }
              return RobotType.ROBOT_2023C;
            } else {
              return robot;
            }
          } else {
            return robot;
          }
        }
      
        public static Mode getMode() {
          switch (getRobot()) {
            case ROBOT_2023C:
            case ROBOT_2023P:
              return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      
            case ROBOT_SIMBOT:
              return Mode.SIM;
      
            default:
              return Mode.REAL;
          }
        }
      
        public static final Map<RobotType, String> logFolders =
            Map.of(RobotType.ROBOT_2023C, "/media/sda2/");
      
        public static enum RobotType {
          ROBOT_2023C,
          ROBOT_2023P,
          ROBOT_SIMBOT
        }
      
        public static enum Mode {
          REAL,
          REPLAY,
          SIM
        }
      
        // Function to disable HAL interaction when running without native libs
        public static boolean disableHAL = false;
      
        public static void disableHAL() {
          disableHAL = true;
        }
      
        /** Checks whether the robot the correct robot is selected when deploying. */
        public static void main(String... args) {
          if (robot == RobotType.ROBOT_SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
            System.exit(1);
          }
        }
      }
      