
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
import frc.robot.commands.Auto.Autonomo;
import frc.robot.commands.swervedrive.MoveAuto.AutonomoControle;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer
{
   private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

   private final PDP pdp = PDP.getInstance();
   
   // Subtitua por CommandPS4Controller ou CommandJoystick se necessário.
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

    // The container for the robot. Contains subsystems, OI devices, and commands.
    
    Trigger tLowBatt = new Trigger(pdp::getLowVoltage);


    public RobotContainer(){
       SmartDashboard.putNumber("Distancia X", x);
       SmartDashboard.putNumber("Distancia Y", y);
       SmartDashboard.putNumber("Direcao", 90);
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

    // Configura os botões do Xbox.
    void configureBindings(){
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


  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }
}