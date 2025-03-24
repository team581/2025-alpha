package frc.robot.util.motion;

public class SyncFollower {
  private final SyncLeader leader;
  private double followerStart = 0;
  private double followerGoal = 0;

  public SyncFollower(SyncLeader leader) {
    this.leader = leader;
  }

  /**
   * Get the setpoint of a follower mechanism, synchronized with the state of the leader.
   *
   * @param goal
   * @return
   */
  public double getSynchronizedSetpoint(double current, double goal) {
    if (goal != followerGoal) {
      followerStart = current;
    }

    followerGoal = goal;

    return leader.getSynchronizedSetpoint(followerStart, goal);
  }
}
