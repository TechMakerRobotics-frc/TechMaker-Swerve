package frc.robot.subsystems;

import java.io.File;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.swerve.TeleopDrive;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

    //swerve drive object
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    @Override
    public void periodic(){}

    @Override
    public void simulationPeriodic(){}

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake (boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        swerveDrive.setModuleStates(desiredStates, false);
     
      }
    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

   



}
