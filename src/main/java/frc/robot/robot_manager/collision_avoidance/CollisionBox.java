package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;

public enum CollisionBox {
  BOX_1(
      1,
      new Rectangle2d( // zone where the station intake happens
          new Translation2d(-20, 14), new Translation2d(-14, 30)),
      new SuperstructurePosition(20, 180)), // whatever it is for station intake
  BOX_2(
      2,
      new Rectangle2d( // bottom right zone
          new Translation2d(-20, 0), new Translation2d(-1, 30)),
      new SuperstructurePosition(0, 180)), // 0 and angle of station intake
  BOX_3(
      3,
      new Rectangle2d( // middle to the left
          new Translation2d(-1, 0), new Translation2d(20, 20)),
      new SuperstructurePosition(0, 45)),

  BOX_4(
      4,
      new Rectangle2d( // middle left zone
          new Translation2d(1, 20), new Translation2d(20, 45)),
      new SuperstructurePosition(30, 45)),
  BOX_5(
      5,
      new Rectangle2d( // top left zone
          new Translation2d(0, 45), new Translation2d(20, 86)),
      new SuperstructurePosition(67, 45)),
  BOX_6(
      6,
      new Rectangle2d( // top right zone
          new Translation2d(-20, 67), new Translation2d(0, 86)),
      new SuperstructurePosition(67, 135));

  public final int id;
  public final Rectangle2d bounds;
  public final SuperstructurePosition safeZone;

  public static CollisionBox getById(int id) {
    return switch (id) {
      case 1 -> BOX_1;
      case 2 -> BOX_2;
      case 3 -> BOX_3;
      case 4 -> BOX_4;
      case 5 -> BOX_5;
      case 6 -> BOX_6;
      default -> BOX_3;
    };
  }

  private CollisionBox(int id, Rectangle2d bounds, SuperstructurePosition safeZone) {
    this.id = id;
    this.bounds = bounds;
    this.safeZone = safeZone;
  }
}
