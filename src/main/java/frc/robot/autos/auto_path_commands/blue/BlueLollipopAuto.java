package frc.robot.autos.auto_path_commands.blue;

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

public class BlueLollipopAuto extends BaseAuto {
  private static final AutoConstraintOptions INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4, 30);
  private static final AutoConstraintOptions BEFORE_INTAKING_CONSTRAINTS =
      new AutoConstraintOptions(1, 57, 4, 30);
  private static final AutoConstraintOptions BEFORE_SCORING_CONSTRAINTS =
      new AutoConstraintOptions(2, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      new AutoConstraintOptions(1.5, 57, 4, 30);

  public BlueLollipopAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_3_AND_4.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(robotManager::rehomeRollRequest),
        // SCORE L4 ON I
        trailblazer
            .followSegment(
                new AutoSegment(
                    BEFORE_SCORING_CONSTRAINTS,
                    new AutoPoint(Points.START_3_AND_4.bluePose, INTAKING_CONSTRAINTS),
                    new AutoPoint(
                        new Pose2d(6.259, 2.952, Rotation2d.fromDegrees(120)),
                        autoCommands
                            .preloadCoralAfterRollHomed()
                            .andThen(autoCommands.l4WarmupCommand(ReefPipe.PIPE_J)),
                        SCORING_CONSTRAINTS),
                    new AutoPoint(robotManager.autoAlign::getUsedScoringPose, SCORING_CONSTRAINTS)),
                false)
            .until(autoCommands::alignedForScore),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE LOLLIPOP 1
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(5.285, 2.368, Rotation2d.fromDegrees(120)),
                    Commands.waitSeconds(0.25).andThen(robotManager::stowRequest)),
                new AutoPoint(
                    new Pose2d(3.906, 2.243, Rotation2d.fromDegrees(180)),
                    autoCommands.floorIntakeUprightCoral(),
                    BEFORE_INTAKING_CONSTRAINTS),
                new AutoPoint(Points.LOLLIPOP_LEFT.bluePose, BEFORE_INTAKING_CONSTRAINTS))),

        // SCORE L4 ON K
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_K)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            BEFORE_SCORING_CONSTRAINTS,
                            new AutoPoint(new Pose2d(3.095, 2.498, Rotation2d.fromDegrees(46.723))),
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose, SCORING_CONSTRAINTS)),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // INTAKE LOLLIPOP 2
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(
                    new Pose2d(3.095, 2.498, Rotation2d.fromDegrees(95.724)),
                    new AutoConstraintOptions()),
                new AutoPoint(
                    new Pose2d(2.936, 4.028, Rotation2d.fromDegrees(180)),
                    autoCommands.floorIntakeUprightCoral(),
                    BEFORE_INTAKING_CONSTRAINTS),
                new AutoPoint(Points.LOLLIPOP_LEFT.bluePose, BEFORE_INTAKING_CONSTRAINTS))),

        // SCORE L4 ON A
        autoCommands
            .l4WarmupCommand(ReefPipe.PIPE_A)
            .alongWith(
                trailblazer
                    .followSegment(
                        new AutoSegment(
                            BEFORE_SCORING_CONSTRAINTS,
                            new AutoPoint(new Pose2d(2.465, 3.809, Rotation2d.fromDegrees(0))),
                            new AutoPoint(new Pose2d(2.746, 3.809, Rotation2d.fromDegrees(0))),
                            new AutoPoint(
                                robotManager.autoAlign::getUsedScoringPose, SCORING_CONSTRAINTS)),
                        false)
                    .until(autoCommands::alignedForScore)),
        autoCommands.l4ScoreAndReleaseCommand(),

        // DRIVE BACK & STOW
        trailblazer.followSegment(
            new AutoSegment(
                INTAKING_CONSTRAINTS,
                new AutoPoint(new Pose2d(2.746, 3.809, Rotation2d.fromDegrees(0))),
                new AutoPoint(
                    new Pose2d(2.465, 3.809, Rotation2d.fromDegrees(0)),
                    autoCommands.stowRequest()))));
  }
}
