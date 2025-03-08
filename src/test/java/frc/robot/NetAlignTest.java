package frc.robot;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.auto_align.NetAlign;
import frc.robot.auto_align.NetAlignState;

public class NetAlignTest {
   @Test
  void scoreInNetOnBlueSideAndFacingNet() {
    var robotPose = new Pose2d(8.55, 0, Rotation2d.fromDegrees(23));

    var result = NetAlign.getAlignState(robotPose, true);

    Assertions.assertEquals(NetAlignState.GOOD, result);
  }

}
