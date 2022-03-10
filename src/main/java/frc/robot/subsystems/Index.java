/** FRC 3603 Index Subsystem
 * Moves the cargo from ready position in the Launcher Subsystem
*/ 

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

// Shuffleboard imports

// CTRE imports
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Index extends SubsystemBase {
  
  // variables
  private final WPI_TalonFX backIndexMotor = new WPI_TalonFX(Constants.BACK_INDEX_MOTOR); // Added so Index() worked
  private final WPI_TalonFX frontIndexMotor = new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR); // Added so Index() worked

  // NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
  private MotorControllerGroup indexMotors = new MotorControllerGroup(
    new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR), 
    new WPI_TalonFX(Constants.BACK_INDEX_MOTOR));
  

  public Index() {
    backIndexMotor.setInverted(true);
    frontIndexMotor.setInverted(false);
  }

  // Starts the Index which moves the cargo
  public void startIndex(double speed){
    indexMotors.set(speed);
  }

  //Sets the Speed of Index Motor
  // 
  double indexVelocity = 3500;
  public void setIndexSpeed() {
    backIndexMotor.set(TalonFXControlMode.Velocity, indexVelocity);
    frontIndexMotor.set(TalonFXControlMode.Velocity, indexVelocity);
  }

  public void stopIndex() {
    indexMotors.set(0);
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