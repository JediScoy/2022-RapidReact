
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

// FIXME one of these motors will need to be inverted or having a negative power output
public class Lift extends SubsystemBase {
  private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
  private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

  private final MotorControllerGroup liftMotors = new MotorControllerGroup(leftLiftMotor, rightLiftMotor); 

public Lift() {
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here
  }

  // Sets the Lift system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  
  public void setLiftSpeed(double speed){
    leftLiftMotor.set(ControlMode.PercentOutput, speed);
    rightLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopLift(){
    leftLiftMotor.set(ControlMode.PercentOutput, 0);
    rightLiftMotor.set(ControlMode.PercentOutput, 0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class

