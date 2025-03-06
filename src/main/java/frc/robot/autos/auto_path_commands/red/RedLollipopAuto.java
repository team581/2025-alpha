package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class RedLollipopAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);

  public RedLollipopAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_2_AND_5.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(Points.START_2_AND_5.redPose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(11.785, 2.0, Rotation2d.fromDegrees(60)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_I)),
                        new AutoConstraintOptions(1.5, 57, 4, 30)),
                    new AutoPoint(
                        robotManager.autoAlign::getUsedScoringPose,
                        new AutoConstraintOptions(1.5, 57, 4, 30))),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE LOLLIPOP 1
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(12.132, 2.243, Rotation2d.fromDegrees(135.88)),
                    Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                new AutoPoint(
                    new Pose2d(13.644, 2.243, Rotation2d.fromDegrees(0)),
                    autoCommands.floorIntakeUprightCoral(),
                    new AutoConstraintOptions(3, 57, 4, 30)),
                new AutoPoint(Points.LOLLIPOP_LEFT.redPose))),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            SCORING_CONSTRAINTS,
                            new AutoPoint(
                                new Pose2d(14.455, 2.498, Rotation2d.fromDegrees(133.277))),
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose,
                                new AutoConstraintOptions(1.5, 57, 4, 30))),
                        false)
                    .until(autoCommands::alignedForScore),
                autoCommands.l4ScoreAndReleaseCommand(),

                // INTAKE LOLLIPOP 2
                trailblazer.followSegment(
                    new AutoSegment(
                        INTAKING_CONSTRAINTS,
                        new AutoPoint(
                            new Pose2d(14.455, 2.498, Rotation2d.fromDegrees(84.276)),
                            new AutoConstraintOptions()),
                        new AutoPoint(
                            new Pose2d(14.614, 4.028, Rotation2d.fromDegrees(0)),
                            autoCommands.floorIntakeUprightCoral()),
                        new AutoPoint(Points.LOLLIPOP_LEFT.redPose))),

                // SCORE L4 ON L
                autoCommands
                    .l4WarmupCommand(ReefPipe.PIPE_L)
                    .alongWith(
                        trailblazer.followSegment(
                            new AutoSegment(
                                SCORING_CONSTRAINTS,
                                new AutoPoint(
                                    new Pose2d(15.085, 3.809, Rotation2d.fromDegrees(180.0))),
                                new AutoPoint(
                                    new Pose2d(14.804, 3.809, Rotation2d.fromDegrees(180))),
                                new AutoPoint(
                                    robotManager.autoAlign::getUsedScoringPose,
                                    new AutoConstraintOptions())),
                            false))
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand());
  }
}
