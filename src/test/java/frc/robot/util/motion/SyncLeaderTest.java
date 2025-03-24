package frc.robot.util.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SyncLeaderTest {
  @Test
  public void test() {
    // 50% complete
    var sync = new SyncLeader();
    sync.update(5, 10);

    var result = sync.getSynchronizedSetpoint(0, 20);

    assertEquals(10, result);
  }
}
