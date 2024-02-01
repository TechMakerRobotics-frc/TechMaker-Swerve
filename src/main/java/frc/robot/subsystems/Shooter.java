
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  boolean extended = false;

  //Dois motores, um de  cada lado 
  CANSparkMax  motorDown = new CANSparkMax(ShooterConstants.kShooterDownMotor,MotorType.kBrushless);
  CANSparkMax  motorUp = new CANSparkMax (ShooterConstants.kShooterUpMotor,MotorType.kBrushless);
  
  public Shooter() {
    
//Limpo qualquer configuração  inicial dos modulos
    motorDown.restoreFactoryDefaults();
    motorUp.restoreFactoryDefaults();

//Configuro para  que o  motor se mantenha estatico quando em 0
    motorDown.setIdleMode(IdleMode.kCoast);
    motorUp.setIdleMode(IdleMode.kCoast);
    
//Configuro a rampa de aceleração para evitar picos de corrente
    motorDown.setOpenLoopRampRate(ShooterConstants.kRampRate);
    motorUp.setOpenLoopRampRate(ShooterConstants.kRampRate);

//Inverto o motor de baixo para que girem juntos
    motorDown.setInverted(false);
    
  }
  public static Shooter getInstance() {
    if (instance == null) {
        instance = new Shooter();
    }
    return instance;
}
  public void setMotorPower(double forward) {
    SmartDashboard.putNumber("Shooter Potencia (%)", forward * 100.0);
      motorUp.set(forward);
      motorDown.set(forward);
    
  }
}
