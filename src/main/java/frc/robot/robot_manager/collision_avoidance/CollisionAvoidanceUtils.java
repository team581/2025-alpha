package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;

public class CollisionAvoidanceUtils {
  private static final double wristLength = 19.0;

  // private static final SuperstructurePosition safePoint2 = new SuperstructurePosition(0.0, 0.0);

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {

    return getGoalPoint(current, goal);
  }

  private static boolean inZone(SuperstructurePosition current, CollisionBoxes collisionBox) {
    Translation2d translation =
        angleHeightToTranslation(current.wristAngle(), current.elevatorHeight());
    Translation2d bottomCorner = collisionBox.box.bottomCorner();
    Translation2d topCorner = collisionBox.box.topCorner();
    if (bottomCorner.getX() < translation.getX()
        && translation.getX() < topCorner.getX()
        && bottomCorner.getY() < translation.getY()
        && translation.getY() < topCorner.getY()) {
      return true;
    }
    return false;
  }

  static CollisionBoxes getZone(SuperstructurePosition current) {
    if (inZone(current, CollisionBoxes.BOX_1)) {
      return CollisionBoxes.BOX_1;
    } else if (inZone(current, CollisionBoxes.BOX_2)) {
      return CollisionBoxes.BOX_2;
    } else if (inZone(current, CollisionBoxes.BOX_3)) {
      return CollisionBoxes.BOX_3;
    } else if (inZone(current, CollisionBoxes.BOX_4)) {
      return CollisionBoxes.BOX_4;
    } else if (inZone(current, CollisionBoxes.BOX_5)) {
      return CollisionBoxes.BOX_5;
    } else if (inZone(current, CollisionBoxes.BOX_6)) {
      return CollisionBoxes.BOX_6;
    } else if (inZone(current, CollisionBoxes.BOX_7)) {
      return CollisionBoxes.BOX_7;
    } else {
      return CollisionBoxes.BAD_BOX;
    }
  }

  private static CollisionBoxes numToCollisionBoxes(int num) {
    if (num == 1) {
      return CollisionBoxes.BOX_1;
    } else if (num == 2) {
      return CollisionBoxes.BOX_2;
    } else if (num == 3) {
      return CollisionBoxes.BOX_3;
    } else if (num == 4) {
      return CollisionBoxes.BOX_4;
    } else if (num == 5) {
      return CollisionBoxes.BOX_5;
    } else if (num == 6) {
      return CollisionBoxes.BOX_6;
    } else return CollisionBoxes.BOX_7;
  }

  static Translation2d angleHeightToTranslation(double wristAngle, double elevatorHeight) {
    return new Translation2d(
        Math.cos(Units.degreesToRadians(wristAngle)) * wristLength,
        elevatorHeight + Math.sin(Units.degreesToRadians(wristAngle)) * wristLength);
  }

  static double distancefromTranslations(
      Translation2d currentTranslation, Translation2d goalTranslation) {
    return Math.sqrt(
        (Math.pow(goalTranslation.getX() - currentTranslation.getX(), 2))
            + (Math.pow(goalTranslation.getY() - currentTranslation.getY(), 2)));
  }

  private static Optional<SuperstructurePosition> getGoalPoint(
      SuperstructurePosition currentSuperstructurePosition,
      SuperstructurePosition goalSuperstructurePosition) {
    CollisionBoxes currentZone = getZone(currentSuperstructurePosition);
    CollisionBoxes goalZone = getZone(goalSuperstructurePosition);

    // if (MathUtil.isNear(goalZone.box.zoneNum(), currentZone.box.zoneNum(), 1)) {
    //   return Optional.empty();
    // }
    if (currentZone.box.zoneNum() < goalZone.box.zoneNum()) {
      return Optional.of(numToCollisionBoxes(currentZone.box.zoneNum() + 1).box.safeZone());
    } else if (currentZone.box.zoneNum() > goalZone.box.zoneNum()) {
      return Optional.of(numToCollisionBoxes(currentZone.box.zoneNum() - 1).box.safeZone());
    }

    return Optional.empty();
  }
}
