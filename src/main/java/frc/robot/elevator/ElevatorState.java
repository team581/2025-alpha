package frc.robot.elevator;

public enum ElevatorState {
  STOWED(0),
  PRE_MATCH_HOMING(0),
  MID_MATCH_HOMING(0),

  INTAKING_CORAL_STATION(0),
  GROUND_CORAL_INTAKE(0),
  GROUND_ALGAE_INTAKE(0),

  ALGAE_DISLODGE_L2(0),
  ALGAE_DISLODGE_L3(0),
  ALGAE_INTAKE_L2(0),
  ALGAE_INTAKE_L3(0),

  NET(0),
  PROCESSOR(0),

  CORAL_L1_PLACE(0),
  CORAL_L2_PLACE(0),
  CORAL_L3_PLACE(0),
  CORAL_L4_PLACE(0),

  CORAL_L1_RELEASE(0),
  CORAL_L2_RELEASE(0),
  CORAL_L3_RELEASE(0),
  CORAL_L4_RELEASE(0),

  UNJAM(0),
  CLIMBING(0),

  COLLISION_AVOIDANCE(0);

  public final double height;

  private ElevatorState(double height) {
    this.height = height;
  }
}
