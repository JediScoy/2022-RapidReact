
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;


public class Lift extends SubsystemBase {
  private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
  private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);

  private final MotorControllerGroup liftMotors = new MotorControllerGroup(leftLiftMotor, rightLiftMotor); 

public void lift() {
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here
  }

  // Sets the Intake system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  
  /** 
  public void setIntakeSpeed(double speed){
    liftMotors.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake(){
    liftMotors.set(ControlMode.PercentOutput, 0);
  }
  */

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


} // End of Class

