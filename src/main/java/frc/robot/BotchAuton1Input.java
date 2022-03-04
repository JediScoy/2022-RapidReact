package frc.robot;

import java.util.function.DoubleSupplier;

public class BotchAuton1Input {
    
public DoubleSupplier x, y, theta;
public double interval; //joint declaration, usable for seperate types of same decoration and type



public BotchAuton1Input(DoubleSupplier x, DoubleSupplier y, double interval) {
    this.x = x;
    this.y = y;
    this.interval = interval;
    
  
}
    
public BotchAuton1Input(DoubleSupplier x, DoubleSupplier y,double interval, DoubleSupplier theta ) {
    this.x = x;
    this.y = y;
    //Allows us to implement rotations later on
    this.theta = theta;
    this.interval = interval;

    
}
    
}
