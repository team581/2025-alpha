package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedLollipopAuto extends BaseAuto {
  public RedLollipopAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        blocks.intakeLollipop(
            new Pose2d(14.5, 2.168, Rotation2d.fromDegrees(0)),
            new Pose2d(15.2, 2.168, Rotation2d.fromDegrees(0))),
        //    new Pose2d(15.0, 2.168, Rotation2d.fromDegrees(160))),
        blocks.scoreL4(ReefPipe.PIPE_A),
        blocks.intakeLollipop(
            new Pose2d(14.929, 3.996, Rotation2d.fromDegrees(0.0)),
            new Pose2d(15.2, 3.996, Rotation2d.fromDegrees(0))),
        //   new Pose2d(15.0, 3.996, Rotation2d.fromDegrees(180))),
        blocks.scoreL4(ReefPipe.PIPE_B));
  }
}
