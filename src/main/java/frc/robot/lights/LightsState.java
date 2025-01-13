package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

public enum LightsState {
  // TODO: Work with Saikiran to finalize these values
  ERROR(Color.kRed, BlinkPattern.BLINK_FAST),
  READY(Color.kGreen, BlinkPattern.BLINK_FAST),
  PLACEHOLDER(Color.kBlack, BlinkPattern.SOLID),
  IN_PROGRESS(Color.kYellow, BlinkPattern.BLINK_SLOW);

  public final BlinkPattern pattern;
  public final Color color;

  LightsState(Color color, BlinkPattern pattern) {
    this.pattern = pattern;
    this.color = color;
  }
}
