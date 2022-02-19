package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Shuffleboard imports
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// CTRE imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Index extends SubsystemBase {
  
  // variables
  // private final TalonFX frontIndexMotor ;  
  // private final TalonFX backIndexMotor;
  // private frontIndexMotor = new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR);
  // backIndexMotor = new WPI_TalonFX(Constants.BACK_INDEX_MOTOR);
  private final WPI_TalonFX backIndexMotor = new WPI_TalonFX(Constants.BACK_INDEX_MOTOR); // Added so Index() worked
  private final WPI_TalonFX frontIndexMotor = new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR); // Added so Index() worked

  // NOTE you must use WPI_TalonFX subclass rather than TalonFX when grouping motors... 60 minutes of my life wasted
  private MotorControllerGroup indexMotors = new MotorControllerGroup(
    new WPI_TalonFX(Constants.FRONT_INDEX_MOTOR), 
    new WPI_TalonFX(Constants.BACK_INDEX_MOTOR));
  
 // private double speed; // schmaybe

  public Index() {
    backIndexMotor.setInverted(true);
    frontIndexMotor.setInverted(false);
  }

  public void startIndex(double speed){
      indexMotors.set(speed);
  }

  public void stopIndex() {
    // frontIndexMotor.set(ControlMode.PercentOutput, 0);
    // backIndexMotor.set(ControlMode.PercentOutput, 0);
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