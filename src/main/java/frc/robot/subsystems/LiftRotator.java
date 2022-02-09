package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class LiftRotator extends SubsystemBase {

// FIXME one of these motors will need to run the opposisite direction. Not sure of the syntax
  public final WPI_TalonFX leftRotationalMotor = new WPI_TalonFX(Constants.LEFT_ROTATIONAL_MOTOR);
  public final WPI_TalonFX rightRotationalMotor = new WPI_TalonFX(
    Constants.RIGHT_ROTATIONAL_MOTOR//, TalonFXInvertType.CounterClockwise
    );


  private final MotorController rotationalLiftMotors = new MotorControllerGroup(leftRotationalMotor, rightRotationalMotor);
    
  
  
  public LiftRotator() {
    rotationalLiftMotors.set(ControlMode.PercentOutput, 0.5);

  }

  public void stopLiftRotational() {
    rotationalLiftMotors.set(ControlMode.PercentOutput, 0);
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