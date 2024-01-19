package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomo extends SequentialCommandGroup {

  
  public Autonomo (SwerveSubsystem drivebase) {
  {
      addCommands(
  new MoveXY(2.1, 0, drivebase),
  new WaitCommand(1),
  new MoveXY(0, 2.2, drivebase),
  new WaitCommand(1),
  new MoveXY(-2.1, 0, drivebase),
  new WaitCommand(1),
  new MoveXY(0, -2.2, drivebase),
  new WaitCommand(1),
  new MoveXY(2.1, 2.2, drivebase),
  new WaitCommand(1),
  new MoveXY(0, -2.1, drivebase));

  // para andar 2metros x = 2.1

  // para andar 2metros y = 2.2
                                        
}
}
}
