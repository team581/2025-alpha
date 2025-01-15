package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefSide {
  SIDE_AB(blueReefSidePose()[0], redReefSidePose()[0]),
  SIDE_CD(blueReefSidePose()[1], redReefSidePose()[1]),
  SIDE_EF(blueReefSidePose()[2], redReefSidePose()[2]),
  SIDE_GH(blueReefSidePose()[3], redReefSidePose()[3]),
  SIDE_IJ(blueReefSidePose()[4], redReefSidePose()[4]),
  SIDE_KL(blueReefSidePose()[5], redReefSidePose()[5]);

  // add a pose of where this side is considered to be
  // used to compare with robot pose to choose closest side
  public final Pose2d bluePose;
  public final Pose2d redPose;

  private static final Pose2d[] bluePoses = {
    new Pose2d(3.658, 4.026, Rotation2d.fromRadians(0)),
    new Pose2d(4.073, 4.746, Rotation2d.fromRadians(-1.048)),
    new Pose2d(4.905, 4.746, Rotation2d.fromRadians(-2.095)),
    new Pose2d(5.321, 4.026, Rotation2d.fromRadians(3.142)),
    new Pose2d(4.905, 3.306, Rotation2d.fromRadians(2.095)),
    new Pose2d(4.074, 3.306, Rotation2d.fromRadians(1.048))
  };
  private static final Pose2d[] redPoses = {
    new Pose2d(13.89, 4.026, Rotation2d.fromRadians(-3.141)),
    new Pose2d(13.475, 3.306, Rotation2d.fromRadians(2.094)),
    new Pose2d(12.643, 3.306, Rotation2d.fromRadians(1.047)),
    new Pose2d(12.227, 4.026, Rotation2d.fromRadians(0)),
    new Pose2d(12.643, 4.746, Rotation2d.fromRadians(-1.047)),
    new Pose2d(13.474, 4.746, Rotation2d.fromRadians(-2.094))
  };

  public static Pose2d[] blueReefSidePose() {
    return bluePoses;
  }

  public static Pose2d[] redReefSidePose() {
    return redPoses;
  }

  public Pose2d getPose() {
    return FmsSubsystem.isRedAlliance() ? redPose : bluePose;
  }

  private ReefSide(Pose2d bluePose, Pose2d redPose) {
      this.bluePose = bluePose;
      this.redPose = redPose;
  }
}
