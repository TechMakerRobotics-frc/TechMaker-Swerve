
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
  CANSparkMax  motorLeft = new CANSparkMax(ShooterConstants.kShooterLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ShooterConstants.kShooterRighrMotor,MotorType.kBrushless);
  
  public Shooter() {
    
//Limpo qualquer configuração  inicial dos modulos
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

//Configuro para  que o  motor se mantenha estatico quando em 0
    motorLeft.setIdleMode(IdleMode.kCoast);
    motorRight.setIdleMode(IdleMode.kCoast);
    
//Configuro a rampa de aceleração para evitar picos de corrente
    motorLeft.setOpenLoopRampRate(ShooterConstants.kRampRate);
    motorRight.setOpenLoopRampRate(ShooterConstants.kRampRate);

//Inverto o motor da esquerda para que girem juntos
    motorLeft.setInverted(true);
    
  }
  public static Shooter getInstance() {
    if (instance == null) {
        instance = new Shooter();
    }
    return instance;
}
  public void setMotorPower(double forward) {
    SmartDashboard.putNumber("Shooter Potencia (%)", forward * 100.0);
      motorRight.set(forward);
      motorLeft.set(forward);
    
  }
}
