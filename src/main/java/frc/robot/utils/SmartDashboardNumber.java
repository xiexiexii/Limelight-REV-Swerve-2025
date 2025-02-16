package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Makes life infinitely easier
public class SmartDashboardNumber {

  // Init stuff
  private double defaultValue;
  private double lastValue;
  private String key;

  // Takes a name and a value and puts it on SmartDashboard
  public SmartDashboardNumber(String key, double defaultValue) {
      this.key = key;
      this.defaultValue = defaultValue;
      this.lastValue = defaultValue;

      SmartDashboard.putNumber(this.key, this.defaultValue);
  }

  // Put the actual number on
  public void putNumber(double val) {
    SmartDashboard.putNumber(this.key, val);
  }

  // Set a default value for the number
  public void setDefaultValue(double val) {
    this.defaultValue = val;
  }

  // Retrieve the value
  public double getNumber() {
    this.lastValue = SmartDashboard.getNumber(this.key, this.defaultValue);
    return this.lastValue;
  }

  // Check if status has changed
  public boolean hasChanged() {
    return Double.compare(this.lastValue, this.getNumber()) != 0;
  }
}