package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;

public class NetAlign {
  private static final double MIDLINE_X = 17.55 / 2.0;
  private static final double TOLERANCE = 0.2;

  public static NetAlignState getAlignState(Pose2d robotPose, boolean isRedAlliance) {
    var robotX = robotPose.getX();
    if (MIDLINE_X - TOLERANCE > robotX) {
      return isRedAlliance ? NetAlignState.TOO_BACKWARD : NetAlignState.TOO_FORWARD;
    }

    if (MIDLINE_X + TOLERANCE < robotX) {
      return isRedAlliance ? NetAlignState.TOO_FORWARD : NetAlignState.TOO_BACKWARD;
    }

    return NetAlignState.GOOD;
  }
}
