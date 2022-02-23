
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; 
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// CTRE imports
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private final WPI_TalonFX leftLiftMotor = new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR);
  private final WPI_TalonFX rightLiftMotor = new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR);
  
  // NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
  private MotorControllerGroup liftMotors = new MotorControllerGroup(
    new WPI_TalonFX(Constants.LEFT_LIFT_MOTOR), 
    new WPI_TalonFX(Constants.RIGHT_LIFT_MOTOR));

public Lift() {
  leftLiftMotor.setInverted(true);
  rightLiftMotor.setInverted(false);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here
    //adding encoder values from lift motors to the dashboard to find out climbing heights
    SmartDashboard.putNumber("Left Lift Arm", leftLiftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Lift Arm", rightLiftMotor.getSelectedSensorPosition());
  }

  // Sets the Lift system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  public void startLift(double speed){
    liftMotors.set(speed);
   }

  public void stopLift(){
    liftMotors.set(0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class