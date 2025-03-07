package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lights.LightsState;

public class NetAlign {
  public static NetAlignState getAlignState(Pose2d robotPose, boolean isRedAlliance) {
    var robotX = robotPose.getX();
    if(isRedAlliance){
      if(robotX > 17.2) {
        LightsState.NET_SCORE_TOO_CLOSE;
     } else if(robotX < 16.9) {
        LightsState.NET_SCORE_TOO_FAR;
     } else {
        LightsState.NET_SCORE_GOOD;
     }
    } else {
       if(robotPose > 17.2) {
      LightsState.NET_SCORE_TOO_CLOSE;
   } else if(robotPose < 16.9) {
      LightsState.NET_SCORE_TOO_FAR;
   } else {
      LightsState.NET_SCORE_GOOD;
   }
    }
  }
  }

