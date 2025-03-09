package frc.robot.vision.game_piece_detection;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.results.GamePieceResult;
import org.junit.jupiter.api.Test;

public class GamePieceDetectionUtilTest {
  @Test
  public void TruffalaPoseTest() {
    var result =
        GamePieceDetectionUtil.calculateFieldRelativeAlgaeTranslationFromCamera(
            new Pose2d(15.41, 4.24, Rotation2d.fromDegrees(32.4)), new GamePieceResult(2.3, 11.8));
    var expected = new Translation2d();
    assertEquals(expected, result);
  }
}
