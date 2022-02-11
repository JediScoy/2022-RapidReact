package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
            false, //drive encoder reversed, change this around if needed
            false, //turn encoder reversed
            Constants.FRONT_LEFT_MODULE_STEER_OFFSET
    
    );

    private final SwerveModule frontRight = new SwerveModule(
            Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
           false,false,
            Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
            

    private final SwerveModule backLeft = new SwerveModule(
            Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_MOTOR,
            false, //drive encoder reversed, change this around if needed
            false, //turn encoder reversed
            Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    private final SwerveModule backRight = new SwerveModule(
            Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
          false,false,
            Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.m_kinematics,
            new Rotation2d(0));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }}