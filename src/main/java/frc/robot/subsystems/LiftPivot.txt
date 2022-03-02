package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Shuffleboard imports
// CTRE imports

public class LiftPivot extends SubsystemBase {

  public final WPI_TalonFX leftPivotMotor = new WPI_TalonFX(Constants.LEFT_PIVOT_MOTOR);
  public final WPI_TalonFX rightPivotMotor = new WPI_TalonFX(Constants.RIGHT_PIVOT_MOTOR //, TalonFXInvertType.CounterClockwise
  );

  private final MotorController liftPivotMotors = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);
  
  public LiftPivot() {
  }

  public void startLiftPivot(double speed) {
    liftPivotMotors.set(speed); 
  }

  public void stopLiftPivot() {
    liftPivotMotors.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class