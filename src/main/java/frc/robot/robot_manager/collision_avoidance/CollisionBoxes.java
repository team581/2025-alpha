package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.BoxZone;
import frc.robot.robot_manager.SuperstructurePosition;

public enum CollisionBoxes {
  BOX_1(
      new BoxZone( // zone where the station intake happens
          1,
          new Translation2d(-1, 20),
          new Translation2d(-20, 30),
          new SuperstructurePosition(20, 180))), // whatever it is for station intake
  BOX_2(
      new BoxZone( // bottom right zone
          2,
          new Translation2d(-1, 0),
          new Translation2d(-20, 20),
          new SuperstructurePosition(0, 180))), // 0 and angle of station intake
  BOX_3(
      new BoxZone( // right in the middle
          3,
          new Translation2d(1, 0),
          new Translation2d(-1, 30),
          new SuperstructurePosition(0, 90))),
  BOX_4(
      new BoxZone( // bottom left zone
          4,
          new Translation2d(-20, 0),
          new Translation2d(-1, 20),
          new SuperstructurePosition(0, 0))),
  BOX_5(
      new BoxZone( // middle left zone
          5,
          new Translation2d(20, 20),
          new Translation2d(1, 45),
          new SuperstructurePosition(30, 45))),
  BOX_6(
      new BoxZone( // top left zone
          6,
          new Translation2d(20, 45),
          new Translation2d(0, 86),
          new SuperstructurePosition(67, 45))),
  BOX_7(
      new BoxZone( // top right zone
          7,
          new Translation2d(0, 67),
          new Translation2d(-20, 86),
          new SuperstructurePosition(67, 180))),
  BAD_BOX(
      new BoxZone(
          0,
          new Translation2d(),
          new Translation2d(),
          new SuperstructurePosition(999, 999) // TODO: fix
          ));

  public final BoxZone box;

  private CollisionBoxes(BoxZone box) {
    this.box = box;
  }
}
