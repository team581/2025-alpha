package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.MathHelpers;

public class MagnetismUtil {
  private static final double IDEAL_MAGNITUDE = 10.0;
  private static final double ASSIST_RADIUS = 2.0;

  private static Pose2d[] getPipePoses() {
    ReefPipe[] values = ReefPipe.values();
    Pose2d[] reefPipes = new Pose2d[12];
    int i = 0;
    for (ReefPipe pipe : values) {
      reefPipes[i] = FmsSubsystem.isRedAlliance() ? pipe.redPose : pipe.bluePose;
      i++;
    }
    return reefPipes;
  }

  private static double nonZeroDivide(double a, double b) {
    return b == 0 ? 0 : a / b;
  }

  public static ChassisSpeeds getMagnetizedChassisSpeeds(ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose, Pose2d goalPose){
    var robotRelativeToPipe = robotPose.relativeTo(goalPose).getTranslation();

    if (goalPose.getTranslation().getDistance(robotPose.getTranslation()) > ASSIST_RADIUS) {
      return fieldRelativeSpeeds;
    }
    var robotSpeeds = MathHelpers.chassisSpeedsToTranslation2d(fieldRelativeSpeeds);
    var idealSpeeds = new Translation2d(IDEAL_MAGNITUDE, robotRelativeToPipe.getAngle());

    var magnetismWeight = (1 - nonZeroDivide(idealSpeeds.getNorm(), ASSIST_RADIUS));

    var unnormalizedTransform =
        idealSpeeds.times(magnetismWeight).plus(robotSpeeds.times(1 - magnetismWeight));

    var normalizedTransform =
        new Translation2d(robotSpeeds.getNorm(), unnormalizedTransform.getAngle());
    return new ChassisSpeeds(
        normalizedTransform.getX(),
        normalizedTransform.getY(),
        fieldRelativeSpeeds.omegaRadiansPerSecond);
  }

  // TODO(Hector): Rename this function to indicate this is used for the reef.
  // Keeping name to not break current integration.
  public static ChassisSpeeds getMagnetizedChassisSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Pose2d robotPose) {
    Pose2d closestReefPipe = robotPose.nearest(AutoAlign.getClosestReefSide(robotPose).getPipes());
    return getMagnetizedChassisSpeeds(fieldRelativeSpeeds, robotPose, closestReefPipe);
  }
}
