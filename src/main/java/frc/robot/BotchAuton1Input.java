package frc.robot;



public class BotchAuton1Input {
    
public double x, y, theta;
public double interval; //joint declaration, usable for seperate types of same decoration and type



public BotchAuton1Input(double x, double y, double interval) {
    this.x = x;
    this.y = y;
    this.interval = interval;
    
  
}
    
public BotchAuton1Input(double x, double y,double interval, double theta ) {
    this.x = x;
    this.y = y;
    //Allows us to implement rotations later on
    this.theta = theta;
    this.interval = interval;

    
}
    
}
