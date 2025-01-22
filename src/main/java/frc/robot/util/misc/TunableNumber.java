package frc.robot.util.misc;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber {
  private String key;
  private double defaultValue;
  private double value;

  private LoggedNetworkNumber loggedDashboardNumber;

  public TunableNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;

    loggedDashboardNumber = new LoggedNetworkNumber("/SmartDashboard/" + key, defaultValue);
  }

  public TunableNumber(String key) {
    this(key, 0.0);
  }

  public TunableNumber(String subsystem, String key, double defaultValue) {
    this.key = "/" + subsystem + "/" + key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;

    loggedDashboardNumber = new LoggedNetworkNumber("/SmartDashboard/" + this.key, defaultValue);
  }

  public double get() {
    return loggedDashboardNumber.get();
  }

  public void set(double value) {
    this.value = value;

    loggedDashboardNumber.set(value);
    ;
  }

  public void reset() {
    set(defaultValue);
  }

  public boolean checkUpdate() {
    double newValue = get();
    if (newValue != value) {
      value = newValue;
      return true;
    }
    return false;
  }

  public void updateFunction(Runnable function) {
    if (checkUpdate()) {
      function.run();
    }
  }
}
