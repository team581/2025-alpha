package frc.robot.util.motion;

import edu.wpi.first.math.MathUtil;

public class SyncLeader {
  private double leaderStart = 0;
  private double leaderGoal = 0;
  private double leaderCurrent = 0;

  /**
   * Update the state of the leader.
   *
   * @param current The current position of the leader.
   * @param goal The goal position of the leader.
   */
  public void update(double current, double goal) {
    if (goal != leaderGoal) {
      leaderStart = current;
    }

    leaderGoal = goal;
    leaderCurrent = current;
  }

  /**
   * Get the setpoint of a follower mechanism, synchronized with the state of the leader.
   *
   * @param start The initial position of the follower when this goal was set.
   * @param goal The goal position of the follower.
   */
  public double getSynchronizedSetpoint(double start, double goal) {
    var t = (leaderCurrent - leaderStart) / (leaderGoal - leaderStart);
    return MathUtil.interpolate(start, goal, t);
  }
}
