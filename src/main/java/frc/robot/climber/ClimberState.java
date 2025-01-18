package frc.robot.climber;

public enum ClimberState {
    UNHOMED(0.0),
    HOMING(0.0),
    STOWED(0.0),
    LINEUP(0.0),
    HANGING(0.0);

    public final double height;

    private ClimberState(double height) {
      this.height = height;
    }
}
