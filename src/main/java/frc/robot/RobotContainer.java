// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.swerve.AbsoluteDrive;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import lib.choreolib.ChoreoSwerveControllerCommand;
import lib.choreolib.ChoreoTrajectory;
import lib.choreolib.TrajectoryManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    CommandXboxController driverXbox = new CommandXboxController(0);

    Field2d m_field = new Field2d();
    ChoreoTrajectory traj;

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                    OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> -driverXbox.getRightX(),
            () -> -driverXbox.getRightY(),
            false);

    TeleopDrive closedTeleopDrive = new TeleopDrive(drivebase,
            () -> driverXbox.getLeftX(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getRightX(),
            () -> driverXbox.getHID().getLeftBumper(),
            false,
            false
    );



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        TrajectoryManager.getInstance().LoadTrajectories();
        traj = TrajectoryManager.getInstance().getTrajectory("New Path.json");
        m_field.getObject("traj").setPoses(
            traj.getInitialPose(), traj.getFinalPose()
        );
        m_field.getObject("trajPoses").setPoses(
            traj.getPoses()
        );
        SmartDashboard.putData(m_field);
        // Configure the trigger bindings
        configureBindings();



        drivebase.setDefaultCommand(closedAbsoluteDrive);
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(exampleSubsystem));

        driverXbox.rightBumper().onTrue(new InstantCommand(drivebase::zeroGyro));

        driverXbox.a().toggleOnTrue(closedAbsoluteDrive);
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        var thetaController =
        new PIDController(
            AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoSwerveControllerCommand swerveControllerCommand =
        new ChoreoSwerveControllerCommand(
            TrajectoryManager.getInstance().getTrajectory("retocurva.json"),
            drivebase::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drivebase::setModuleStates,
            true,
            drivebase);

    // Reset odometry to the starting pose of the trajectory.
    drivebase.resetOdometry(traj.getInitialPose());

    // Run path following command, then stop at the end.
    return Commands.sequence(
        Commands.runOnce(()->drivebase.resetOdometry(traj.getInitialPose())),
        swerveControllerCommand,
        Commands.runOnce(() -> drivebase.lock())
    );
    }
    
  public void periodic() {
        m_field.setRobotPose(drivebase.getPose());
      }
    
}
