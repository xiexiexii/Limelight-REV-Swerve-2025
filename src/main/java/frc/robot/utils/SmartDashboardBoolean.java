package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Makes life infinitely easier
public class SmartDashboardBoolean {

  // Init Stuff
  private boolean defaultValue;
  private boolean lastValue;
  private String key;

  // Takes a name and boolean and puts it on SmartDashboard
  public SmartDashboardBoolean(String key, boolean defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;

    SmartDashboard.putBoolean(this.key, this.defaultValue);
  }

  // Puts boolean value on
  public void putNumber(boolean val) {
    SmartDashboard.putBoolean(this.key, val);
  }

  // Sets a default value for the boolean
  public void setDefaultValue(boolean val) {
    this.defaultValue = val;
  }

  // Retrieve the boolean
  public boolean getValue() {
    this.lastValue = SmartDashboard.getBoolean(this.key, this.defaultValue);
    return this.lastValue;
  }

  // Check if status has changed
  public boolean hasChanged() {
    return this.lastValue != this.getValue();
  }
}