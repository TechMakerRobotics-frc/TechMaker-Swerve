// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.Constants.Auton;

public class MoveXY extends CommandBase {
  /** Creates a new MoveStraight. */
  double distanceX, distanceY;
  SwerveSubsystem swerve;
  boolean finish = false;
  private final SwerveController controller;
  double lastTimestamp;
  public MoveXY(double distanceX, double distanceY, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    this.swerve = swerve;
    this.controller = swerve.getSwerveController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOdometry();
    swerve.zeroGyro();
    SmartDashboard.putString("Ja acabou", "NAO");
    lastTimestamp = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speedX = 0;
    double speedY = 0;
    
    finish = true;
    if(Math.abs(swerve.getPose().getX())<Math.abs(distanceX))
    {
      finish = false;

    }
    if(Math.abs(swerve.getPose().getY())<Math.abs(distanceY))
    {
      speedY = 0.5;
      Math.copySign(speedY, distanceY);
      finish = false;

    }
    // Cálculos -P-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = Auton.kp*errorX;
    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(0, 3);
    double angVelocity = Math.pow(0, 3);

    // Cálculos -I-
    double errorSumX = 0;
    
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    errorSumX += errorX * dt;

    speedX = Auton.kp * errorX + Auton.ki * errorSumX;
    lastTimestamp = Timer.getFPGATimestamp();

    /*double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = Auton.kp*errorY; */

    SmartDashboard.putNumber("sensorX", sensorX);
    SmartDashboard.putNumber("errorX", errorX);
    SmartDashboard.putNumber("speedX", speedX);
    SmartDashboard.putNumber("xVelocity", xVelocity);
    
    
  
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 true ,false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.lock();
    SmartDashboard.putString("Ja acabou", "SIM");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
