package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefPipe {
  PIPE_A(
      new Pose2d(3.7086032, 3.8616128, new Rotation2d(Math.PI)),
      new Pose2d(13.8396472, 4.1901872, new Rotation2d(0.0))),
  PIPE_B(
      new Pose2d(3.7086032, 4.190238, new Rotation2d(Math.PI)),
      new Pose2d(13.8396472, 3.861562, new Rotation2d(0.0))),
  PIPE_C(
      new Pose2d(3.95667691423813, 4.619914277082763, new Rotation2d(2.0943951023931953)),
      new Pose2d(13.591573485761872, 3.4318857229172375, new Rotation2d(-1.047197551196598))),
  PIPE_D(
      new Pose2d(4.241274685761871, 4.784226877082762, new Rotation2d(2.0943951023931953)),
      new Pose2d(13.30697571423813, 3.267573122917238, new Rotation2d(-1.047197551196598))),
  PIPE_E(
      new Pose2d(4.73742211423813, 4.784226877082762, new Rotation2d(1.0471975511965976)),
      new Pose2d(12.810828285761872, 3.267573122917238, new Rotation2d(-2.0943951023931957))),
  PIPE_F(
      new Pose2d(5.022019885761871, 4.619914277082763, new Rotation2d(1.0471975511965976)),
      new Pose2d(12.52623051423813, 3.4318857229172375, new Rotation2d(-2.0943951023931957))),
  PIPE_G(
      new Pose2d(5.2700936, 4.190238, new Rotation2d(0.0)),
      new Pose2d(12.2781568, 3.861562, new Rotation2d(Math.PI))),
  PIPE_H(
      new Pose2d(5.2700936, 3.8616128, new Rotation2d(0.0)),
      new Pose2d(12.2781568, 4.1901872, new Rotation2d(Math.PI))),
  PIPE_I(
      new Pose2d(5.022019885761871, 3.43193652291724, new Rotation2d(-1.0471975511965976)),
      new Pose2d(12.52623051423813, 4.619863477082762, new Rotation2d(2.0943951023931957))),
  PIPE_J(
      new Pose2d(4.73742211423813, 3.26762392291724, new Rotation2d(-1.0471975511965976)),
      new Pose2d(12.810828285761872, 4.784176077082762, new Rotation2d(2.0943951023931957))),
  PIPE_K(
      new Pose2d(4.241274685761871, 3.26762392291724, new Rotation2d(-2.0943951023931953)),
      new Pose2d(13.30697571423813, 4.784176077082762, new Rotation2d(1.0471975511965979))),
  PIPE_L(
      new Pose2d(3.95667691423813, 3.43193652291724, new Rotation2d(-2.0943951023931953)),
      new Pose2d(13.591573485761872, 4.619863477082762, new Rotation2d(1.0471975511965979)));

  public final Pose2d redPose;
  public final Pose2d bluePose;

  ReefPipe(Pose2d bluePose, Pose2d redPose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  public Pose2d getPose() {
    return (FmsSubsystem.isRedAlliance() ? redPose : bluePose)
        .rotateBy(Rotation2d.fromRotations(0.5));
  }
}
