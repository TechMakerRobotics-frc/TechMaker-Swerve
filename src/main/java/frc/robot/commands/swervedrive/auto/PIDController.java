package frc.robot.commands.swervedrive.auto;
 
public class PIDController {
  private double kp;  // Ganho proporcional
  private double ki;  // Ganho integral
  private double kd;  // Ganho derivativo

  private double setpoint;  // Valor desejado
  private double integral;   // Soma acumulativa dos erros
  private double lastError;  // Último erro

  public PIDController(double kp, double ki, double kd) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
  }
  public double calculate(double currentValue) {
      double error = setpoint - currentValue;
      integral += error;
      double derivative = error - lastError;

      // Fórmula PID
      double output = kp * error + ki * integral + kd * derivative;

      lastError = error;
      return output;
  }
}
