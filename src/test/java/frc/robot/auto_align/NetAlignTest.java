package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class NetAlignTest {
  @Test
  void testAlignRed() {
    var robotPose = new Pose2d(8.6, 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, true);

    Assertions.assertEquals(NetAlignState.GOOD, result);
  }

  @Test
  void testAlignRedForward() {
    var robotPose = new Pose2d(9.0, 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, true);

    Assertions.assertEquals(NetAlignState.TOO_FORWARD, result);
  }

  @Test
  void testAlignRedBackward() {
    var robotPose = new Pose2d(8.3, 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, true);

    Assertions.assertEquals(NetAlignState.TOO_BACKWARD, result);
  }

  @Test
  void testAlignBlue() {
    var robotPose = new Pose2d(8.8, 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, false);

    Assertions.assertEquals(NetAlignState.GOOD, result);
  }

  @Test
  void testAlignBlueForward() {
    var robotPose = new Pose2d(8.0, 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, false);

    Assertions.assertEquals(NetAlignState.TOO_FORWARD, result);
  }

  @Test
  void testAlignBlueBackward() {
    var robotPose = new Pose2d(10., 0, Rotation2d.fromDegrees(0));

    var result = NetAlign.getAlignState(robotPose, false);

    Assertions.assertEquals(NetAlignState.TOO_BACKWARD, result);
  }
}
