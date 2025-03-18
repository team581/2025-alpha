package frc.robot.auto_align;

public enum ReefAlignState {
  ALL_CAMERAS_DEAD,

  // Checking based on tags
  NO_TAGS_WRONG_POSITION,
  NO_TAGS_IN_POSITION(true),
  HAS_TAGS_WRONG_POSITION,
  HAS_TAGS_IN_POSITION(true);


  private final boolean inPosition;
  private ReefAlignState() {
    this.inPosition = false;
  }
  private ReefAlignState(boolean inPosition) {
    this.inPosition = inPosition;
  }

  public boolean getInPosition() {
    return inPosition;
  }
}
