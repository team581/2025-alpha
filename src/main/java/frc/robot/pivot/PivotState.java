package frc.robot.pivot;

public enum PivotState {
  HOMING(0.0),

  CORAL_SCORE(0.0),

  STOWED(0.0),
  CORAL_LEFT(-90),
  CORAL_RIGHT(90);

  public final double angle;

  PivotState(double angle) {
    this.angle = angle;
  }
}
