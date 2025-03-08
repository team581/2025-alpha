package frc.robot.climber;

public enum ClimberState {
  STOWED(195, 2, -2),
  LINEUP(0, 4, -4),
  HANGING(156.0, 4, -4),
  HANGING_2(160.0, 4, -4),
  HANGING_3(165.0, 4, -4);

  public final double angle;
  public final double forwardsVoltage;
  public final double backwardsVoltage;

  private ClimberState(double angle, double forwardVoltage, double backwardsVoltage) {
    this.angle = angle;
    this.forwardsVoltage = forwardVoltage;
    this.backwardsVoltage = backwardsVoltage;
  }
}
