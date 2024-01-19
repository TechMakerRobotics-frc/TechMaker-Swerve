package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomo extends SequentialCommandGroup {

  
  public Autonomo (SwerveSubsystem drivebase) {
  {
      addCommands(
  new MoveXY(2.1, 0, drivebase),
  new WaitCommand(10));
}
}
}
