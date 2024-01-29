// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.Autonomo;
import frc.robot.commands.swervedrive.auto.AutonomoControle;
//import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // private AprilTagVision aprilTagVision;
  // Subsystem ^

   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
   
   // Replace with CommandPS4Controller or CommandJoystick if needed
   CommandXboxController driverXbox = new CommandXboxController(0);

    
TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_X_DEADBAND),
        () -> (driverXbox.getRawAxis(2)-driverXbox.getRawAxis(3)), () -> true);


        double x = 2.7;
        double y = 2.7;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
       SmartDashboard.putNumber("Distancia X", x);
       SmartDashboard.putNumber("Distancia Y", y);
       SmartDashboard.putNumber("Direcao", 90);
        // Configure the trigger bindings
       
        /*if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
          arm = new Arm(new ArmIOSparkMax(), new ArmSolverIOKairos(4));
          gripper = new Gripper(new GripperIOSparkMax());
          cubeIntake = new CubeIntake(new CubeIntakeIOSparkMax());
          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIONorthstar("northstar_0"),
                  new AprilTagVisionIONorthstar("northstar_1"),
                  new AprilTagVisionIONorthstar("northstar_2"),
                  new AprilTagVisionIONorthstar("northstar_3"));
          objectiveTracker = new ObjectiveTracker(new NodeSelectorIOServer());
          break; */

        
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     //* CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    void configureBindings()
    {
       
        driverXbox.rightBumper().onTrue(new InstantCommand(drivebase::zeroGyro));
        driverXbox.leftBumper().onTrue(new InstantCommand(drivebase::resetOdometry));
        driverXbox.a().onTrue(new InstantCommand(drivebase::lock));
        driverXbox.b().onTrue(new AutonomoControle(drivebase));
        drivebase.setDefaultCommand(closedFieldRel);
    }
  
    
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {
        return new Autonomo(drivebase);
    
  }


  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }
}
/*
Código para apriltag seguindo do RobotCode 2023

if (aprilTagVision == null) {
      // In replay, match the number of instances for each robot
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {});
          break;
        case ROBOT_2023P:
          aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
          break;
        default:
          aprilTagVision = new AprilTagVision();
          break;
      }
    } */
