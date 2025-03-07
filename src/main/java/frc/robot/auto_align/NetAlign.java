package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lights.LightsState;

public class NetAlign {
  public static NetAlignState getAlignState(Pose2d robotPose, boolean isRedAlliance) {
    var robotX = robotPose.getX();
    if(isRedAlliance){
      if(robotX > 17.2) {
         return NetAlignState.TOO_FORWARD;
     } else if(robotX < 16.9) {
        return NetAlignState.TOO_BACKWARD;
     } else {
        return NetAlignState.GOOD;
     }
    } else {
       if(robotX > 17.2) {
        return NetAlignState.TOO_FORWARD;
   } else if(robotX < 16.9) {
    return NetAlignState.TOO_BACKWARD;
   } else {
    return NetAlignState.GOOD;
   }
    }
  }
  }

