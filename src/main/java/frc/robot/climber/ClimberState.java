package frc.robot.climber;

public enum ClimberState {
  STOWED(80.0, 2, -2),
  LINEUP(0.0, 3, -3),
  HANGING(40.0, 3, -3),
  HANGING_2(60.0, 3, -3),
  HANGING_3(80.0, 3, -3);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
