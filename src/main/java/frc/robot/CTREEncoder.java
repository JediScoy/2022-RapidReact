package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CTREEncoder {

    WPI_TalonFX talon;
    boolean invert;
    double distancePerPulse;

    public CTREEncoder(WPI_TalonFX _talon, boolean invert, double distancePerPulse) {
        talon = _talon;
        talon.getSensorCollection();
        this.invert = invert;
        this.distancePerPulse = distancePerPulse;
    }

    public double getRate() {
        return talon.getSelectedSensorVelocity() * distancePerPulse; //this doesn't smell right
    }
//hopefully adding an extra method doesnt break the funcitonality of the class entirely
    public int getChannel(){//FIXME: what is the difference between getdeviceID() and getBaseID()? (and more importantly, which one is correct?)
        return talon.getDeviceID();
    }
    public double getDistance() {
        return talon.getSelectedSensorPosition() * distancePerPulse;
    }

    public double get() {
        return invert ? -talon.getSelectedSensorPosition() : talon.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return invert ? -talon.getSelectedSensorVelocity() : talon.getSelectedSensorVelocity();
    }

    public void setPosition(double pos) {
        talon.setSelectedSensorPosition(pos);
    }

    public void reset() {
        setPosition(0);
    }
}