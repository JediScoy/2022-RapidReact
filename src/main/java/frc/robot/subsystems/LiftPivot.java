package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports

public class LiftPivot extends SubsystemBase {

// FIXME one of these motors will need to run the opposisite direction. Not sure of the syntax
  public final WPI_TalonFX leftPivotMotor = new WPI_TalonFX(Constants.LEFT_PIVOT_MOTOR);
  public final WPI_TalonFX rightPivotMotor = new WPI_TalonFX(Constants.RIGHT_PIVOT_MOTOR //, TalonFXInvertType.CounterClockwise
  );


  private final MotorController liftPivotMotors = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);
  
  public LiftPivot() {
    leftPivotMotor.set(ControlMode.PercentOutput, 0.5); 
    rightPivotMotor.set(ControlMode.PercentOutput, 0.5);

  }

  public void stopLiftRotational() {
    // FIXME rotationalLiftMotors.set(ControlMode.PercentOutput, 0);
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