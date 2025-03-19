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
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R3_AND_B3.redPose, ReefPipe.PIPE_I),
        blocks.intakeLollipop(
            new Pose2d(14.431, 2.168, Rotation2d.fromDegrees(0.0)),
            new Pose2d(15.700, 2.168, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_A),
        blocks.intakeLollipop(
            new Pose2d(15.006, 3.996, Rotation2d.fromDegrees(0.0)),
            new Pose2d(15.734, 3.996, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_B));
  }
}
