package frc.robot.purple;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.limelight.Limelight;

public class Purple {
  private final Limelight camera;
  private static final double ALIGN_KP = 3.0;

  public Purple(Limelight camera) {
    this.camera = camera;
  }

  public PurpleState getPurpleState() {
    var maybeResult = camera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return PurpleState.NO_PURPLE;
    }

    var result = maybeResult.get();

    if (MathUtil.isNear(0, result.ty(), 1)) {
      return PurpleState.CENTERED;
    }

    return PurpleState.VISIBLE_NOT_CENTERED;
  }

  public ChassisSpeeds getPurpleAlignChassisSpeeds(double robotHeading) {
    var maybeResult = camera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return new ChassisSpeeds();
    }
    var rawAngle = camera.getPurpleResult().get().ty();
    DogLog.log("Purple/RawAngleTY", rawAngle);
    var rawAngleRadians = Units.degreesToRadians(rawAngle);
    var rawAngleTranslation = new Translation2d(0, rawAngleRadians);
    var rotatedAngleTranslation =
        rawAngleTranslation.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = rotatedAngleTranslation.getX();
    var yError = rotatedAngleTranslation.getY();

    var xEffort = xError * ALIGN_KP;
    var yEffort = yError * ALIGN_KP;
    DogLog.log("Purple/XEffort", xEffort);
    DogLog.log("Purple/YEffort", yEffort);
    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }
}
