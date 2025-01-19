package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.purple.PurpleState;
import frc.robot.vision.limelight.Limelight;
import java.util.List;

public class AutoAlign {
  private static final List<ReefSide> ALL_REEF_SIDES = List.of(ReefSide.values());
  private final Limelight camera;

  public AutoAlign(Limelight camera) {
    this.camera = camera;
  }

  public static Pose2d getClosestReefSide(Pose2d robotPose) {
    var reefSide =
        ALL_REEF_SIDES.stream()
            .min(
                (a, b) ->
                    Double.compare(
                        robotPose.getTranslation().getDistance(a.getPose().getTranslation()),
                        robotPose.getTranslation().getDistance(b.getPose().getTranslation())))
            .get();
    return reefSide.getPose();
  }

  public static boolean shouldNetScoreForwards(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees();

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      return theta < 90 || theta > 270;
    }

    // Robot is on red side
    return theta > 90 && theta < 270;
  }

  public static boolean isCloseToReefSide(Pose2d robotPose, Pose2d nearestReefSide) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < Units.feetToMeters(3);
  }

  public static ReefAlignState getReefAlignState(
      Pose2d robotPose, PurpleState purpleState, Limelight camera) {
    Pose2d reefPose = getClosestReefSide(robotPose);

    if (camera.getInterpolatedTagResult().isEmpty() && purpleState == PurpleState.NO_PURPLE) {
      return ReefAlignState.NO_TAGS_WRONG_POSITION;
    }
    if (purpleState == PurpleState.CENTERED) {
      return ReefAlignState.HAS_PURPLE_ALIGNED;
    }
    if (purpleState == PurpleState.VISIBLE_NOT_CENTERED) {
      return ReefAlignState.HAS_PURPLE_NOT_ALIGNED;
    }
    if (camera.getInterpolatedTagResult().isPresent() && !isCloseToReefSide(robotPose, reefPose)) {
      return ReefAlignState.HAS_TAGS_WRONG_POSITION;
    } else if (isCloseToReefSide(robotPose, reefPose)) {
      return ReefAlignState.HAS_TAGS_IN_POSITION;
    }
    return ReefAlignState.CAMERA_DEAD;
  }
}
