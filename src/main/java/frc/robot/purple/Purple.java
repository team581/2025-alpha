package frc.robot.purple;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldUtil;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.vision.LimelightHelpers;

public class Purple {
  private static final double ASSIST_KP = 0.1;
  private static final String LIMELIGHT_NAME = "asdfkj";
  private static final double MAX_ANGLE_CHANGE = -35.0;
  private static final double MIN_ANGLE_CHANGE = 35.0;

  private final LocalizationSubsystem localization;

  private Pose2d closestScoreSpot;

  public Purple(LocalizationSubsystem localization) {

    this.localization = localization;
  }

  /// tune angle change
  private double distanceFromScoreSpot(Pose2d robotPose, Pose2d scorePose) {
    return Math.sqrt(
        Math.pow((robotPose.getX() - scorePose.getX()), 2)
            + Math.pow((robotPose.getY() - scorePose.getY()), 2));
  }

  private Pose2d getScoreSpot() {
    closestScoreSpot = FieldUtil.getReefSpot1();
    for (int i = 0; i <= 12; ) {
      if (distanceFromScoreSpot(localization.getPose(), closestScoreSpot)
          > distanceFromScoreSpot(localization.getPose(), FieldUtil.getReefSpot(i))) {
        closestScoreSpot = FieldUtil.getReefSpot(i);
      }
      i++;
    }

    return closestScoreSpot;
  }

  public ChassisSpeeds getRobotRelativeScoreAssistSpeeds(
      double robotHeading, ChassisSpeeds fieldRelativeInputSpeeds) {

    double angle =
        Units.radiansToDegrees(
            Math.atan2(
                getScoreSpot().getY() - localization.getPose().getY(),
                getScoreSpot().getX() - localization.getPose().getX()));
    double angleChange = MathUtil.clamp(MIN_ANGLE_CHANGE, MAX_ANGLE_CHANGE, angle * ASSIST_KP);

    Translation2d requestedFieldRelativeDrive =
        new Translation2d(
            fieldRelativeInputSpeeds.vxMetersPerSecond, fieldRelativeInputSpeeds.vyMetersPerSecond);

    Translation2d newDriveRequest =
        requestedFieldRelativeDrive.rotateBy(Rotation2d.fromDegrees(angleChange));

    return new ChassisSpeeds(
        newDriveRequest.getX(),
        newDriveRequest.getY(),
        fieldRelativeInputSpeeds.omegaRadiansPerSecond);
  }

  private static final double X_ADJUST_P = 0.1;

  public static ChassisSpeeds getPurpleAdjustmentRobotRelative() {
    var tx = LimelightHelpers.getTX("asdasd");

    if (tx == 0) {
      return new ChassisSpeeds();
    }

    var error = getXError(tx);

    var adjustment = getRobotAdjustmentRobotRelative(error);

    return adjustment;
  }

  private static double getXError(double tx) {
    return -tx;
  }

  private static ChassisSpeeds getRobotAdjustmentRobotRelative(double xError) {
    // adjust side to side, don't care about forward/backward, don't care about rotation
    double xTranslation = X_ADJUST_P * xError;
    ChassisSpeeds translation = new ChassisSpeeds(xTranslation, 0, 0);
    return translation;
  }
}
