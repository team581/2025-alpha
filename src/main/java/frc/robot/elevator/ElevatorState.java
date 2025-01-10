package frc.robot.elevator;

public enum ElevatorState {
  STOWED(0),
  CLIMBING(999),
  INTAKE_CORAL_STATION(999),
  INTAKE_CORAL_FLOOR(999),
  INTAKE_ALGAE_FLOOR(999),
  UNJAM(999),
  ALGAE_DISLODGE_L2(999),
  ALGAE_DISLODGE_L3(999),
  NET(999),
  CORAL_L1(999),
  CORAL_L2(999),
  CORAL_L3(999),
  CORAL_L4(999);
  final int value;

  private ElevatorState(int position) {
    this.value = position;
  }
}
