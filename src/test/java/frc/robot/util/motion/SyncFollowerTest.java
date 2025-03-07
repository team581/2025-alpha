package frc.robot.util.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SyncFollowerTest {
  @Test
  public void test() {
    // 50% complete
    var leader = new SyncLeader();
    leader.update(0, 10);
    leader.update(5, 10);

    var follower = new SyncFollower(leader);

    var result = follower.getSynchronizedSetpoint(10, 20);

    assertEquals(15, result);
  }
}
