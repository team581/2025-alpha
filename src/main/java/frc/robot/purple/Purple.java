package frc.robot.purple;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.autos.followers.PathFollower;
import frc.robot.vision.limelight.Limelight;

public class Purple {
  private final Limelight purpleCamera;
  private final Limelight frontTagCamera;
  private static final double PURPLE_SIDEWAYS_KP = 3.0;
  private static final double TAG_FORWARDS_KP = 3.0;
  private static final double ROBOT_FORWARDS_OFFSET_FROM_REEF_SIDE = 0.6;

  public Purple(Limelight purpleCamera, Limelight frontTagCamera) {
    this.purpleCamera = purpleCamera;
    this.frontTagCamera = frontTagCamera;
  }

  public PurpleState getPurpleState() {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return PurpleState.NO_PURPLE;
    }

    var result = maybeResult.get();

    if (MathUtil.isNear(0, result.ty(), 1)) {
      return PurpleState.CENTERED;
    }

    return PurpleState.VISIBLE_NOT_CENTERED;
  }

  public ChassisSpeeds getTagAlignChassisSpeeds(double robotHeading) {
    var maybePose = frontTagCamera.getRobotRelativePoseToTag();
    if (maybePose.isEmpty()) {
      return new ChassisSpeeds();
    }
    DogLog.log("PurpleAlignment/Tag/Pose", maybePose.get());

    var forwardOffset = (-1 * maybePose.get().getX()) - ROBOT_FORWARDS_OFFSET_FROM_REEF_SIDE;
    var forwardOffsetTranslation = new Translation2d(forwardOffset, 0.0);
    var forwardOffsetTranslationRotated =
        forwardOffsetTranslation.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = forwardOffsetTranslationRotated.getX();
    var yError = forwardOffsetTranslationRotated.getY();
    DogLog.log("PurpleAlignment/Tag/XError", xError);
    DogLog.log("PurpleAlignment/Tag/YError", yError);

    var xEffort = xError * TAG_FORWARDS_KP;
    var yEffort = yError * TAG_FORWARDS_KP;
    DogLog.log("PurpleAlignment/Tag/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Tag/YEffort", yEffort);
    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }

  // public ChassisSpeeds getPoseAlignChassisSpeeds(Pose2d robotPose, ReefPipeLevel level) {
  //   var pipePose = AutoAlign.getClosestReefPipe(robotPose, level);
  //   var pipePoseRobotRelative = pipePose.getTranslation().rotateBy(Rotation2d.fromDegrees(360-robotPose.getRotation().getDegrees())).minus(robotPose.getTranslation());
  //   var forwardOffset = pipePoseRobotRelative.getX();
  //   var forwardOffsetTranslation = new Translation2d(forwardOffset, 0.0);
  //   var forwardOffsetTranslationRotated =
  //       forwardOffsetTranslation.rotateBy(Rotation2d.fromDegrees(robotPose.getRotation().getDegrees()));
  //   var xError = forwardOffsetTranslationRotated.getX();
  //   var yError = forwardOffsetTranslationRotated.getY();
  //   DogLog.log("PurpleAlignment/Tag/XError", xError);
  //   DogLog.log("PurpleAlignment/Tag/YError", yError);

  //   var xEffort = xError * TAG_FORWARDS_KP;
  //   var yEffort = yError * TAG_FORWARDS_KP;
  //   DogLog.log("PurpleAlignment/Tag/XEffort", xEffort);
  //   DogLog.log("PurpleAlignment/Tag/YEffort", yEffort);
  //   return new ChassisSpeeds(xEffort, yEffort, 0.0);
  // }


  public ChassisSpeeds getPurpleAlignChassisSpeeds(double robotHeading) {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return new ChassisSpeeds();
    }
    var rawAngle = purpleCamera.getPurpleResult().get().ty();
    DogLog.log("PurpleAlignment/Purple/RawAngleTY", rawAngle);
    var rawAngleRadians = Units.degreesToRadians(rawAngle);
    var rawAngleTranslation = new Translation2d(0, rawAngleRadians);
    var rotatedAngleTranslation =
        rawAngleTranslation.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = rotatedAngleTranslation.getX();
    var yError = rotatedAngleTranslation.getY();

    var xEffort = xError * PURPLE_SIDEWAYS_KP;
    var yEffort = yError * PURPLE_SIDEWAYS_KP;
    DogLog.log("PurpleAlignment/Purple/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Purple/YEffort", yEffort);
    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }

  public ChassisSpeeds getCombinedTagAndPurpleChassisSpeeds(double robotHeading) {
    var tagSpeeds = getTagAlignChassisSpeeds(robotHeading);
    var purpleSpeeds = getPurpleAlignChassisSpeeds(robotHeading);
    var combinedSpeeds = tagSpeeds.plus(purpleSpeeds);
    DogLog.log("PurpleAlignment/Combined/XEffort", combinedSpeeds.vxMetersPerSecond);
    DogLog.log("PurpleAlignment/Combined/YEffort", combinedSpeeds.vyMetersPerSecond);
    return combinedSpeeds;
  }
}
