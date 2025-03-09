package frc.robot.climber;

public enum ClimberState {
  STOWED(80.0, 2, -2),
  LINEUP(0.0, 12, -12),
  HANGING(40.0, 12, -12),
  HANGING_2(60.0, 12, -12),
  HANGING_3(83.0, 12, -12);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
