package frc.robot.auto_align;

import java.util.EnumSet;

public class ReefState {
  private EnumSet<ReefPipe> scoredL2Pipes = EnumSet.noneOf(ReefPipe.class);
  private EnumSet<ReefPipe> scoredL3Pipes = EnumSet.noneOf(ReefPipe.class);
  private EnumSet<ReefPipe> scoredL4Pipes = EnumSet.noneOf(ReefPipe.class);


  public void clear() {
    scoredL2Pipes.clear();
    scoredL3Pipes.clear();
    scoredL4Pipes.clear();
  }

  public void markScored(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L2:
        scoredL2Pipes.add(pipe);
        break;
      case L3:
        scoredL3Pipes.add(pipe);
        break;
      case L4:
        scoredL4Pipes.add(pipe);
        break;
      default:
        break;
    }
  }

  public boolean isScored(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L2:
        return scoredL2Pipes.contains(pipe);
      case L3:
        return scoredL3Pipes.contains(pipe);
      case L4:
        return scoredL4Pipes.contains(pipe);
      default:
        return false;
    }
  }
}
