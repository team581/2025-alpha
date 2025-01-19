package frc.robot.vision;

public enum CameraHealth {
  GOOD,
  OFFLINE,
  NO_TARGETS;

  public static CameraHealth combine(CameraHealth a, CameraHealth b) {
    if (a == CameraHealth.OFFLINE && b == CameraHealth.OFFLINE) {
      return CameraHealth.OFFLINE;
    }
    if (a == CameraHealth.GOOD || b == CameraHealth.GOOD) {
      return CameraHealth.GOOD;
    }
    return CameraHealth.NO_TARGETS;
  }
}
