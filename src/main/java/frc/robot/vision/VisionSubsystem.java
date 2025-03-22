package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.ReefPipe;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.TagResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends StateMachine<VisionState> {
  private static final double REEF_CLOSEUP_DISTANCE = 1.0;
  private final ImuSubsystem imu;
  private final Limelight backTagLimelight;
  private final Limelight frontRightLimelight;
  private final Limelight frontLeftLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;
  private ReefPipe reefPipe;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight backTagLimelight,
      Limelight frontRightLimelight,
      Limelight frontLeftLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.backTagLimelight = backTagLimelight;
    this.frontRightLimelight = frontRightLimelight;
    this.frontLeftLimelight = frontLeftLimelight;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getRobotHeading();
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    if (getState() == VisionState.CLOSEST_REEF_TAG_CLOSEUP) {
      switch (reefPipe) {
        case PIPE_A, PIPE_C, PIPE_E, PIPE_G, PIPE_I, PIPE_K -> {
          if (frontLeftLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            frontLeftLimelight.setState(LimelightState.OFF);
            frontRightLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
        case PIPE_B, PIPE_D, PIPE_F, PIPE_H, PIPE_J, PIPE_L -> {
          if (frontRightLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            frontRightLimelight.setState(LimelightState.OFF);
            frontLeftLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
      }
    }

    tagResult.clear();
    var maybeBackResult = backTagLimelight.getTagResult();
    var maybeFrontRightResult = frontRightLimelight.getTagResult();
    var maybeFrontLeftResult = frontLeftLimelight.getTagResult();

    if (maybeBackResult.isPresent()) {
      tagResult.add(maybeBackResult.orElseThrow());
    }

    if (maybeFrontRightResult.isPresent()) {
      tagResult.add(maybeFrontRightResult.orElseThrow());
    }

    if (maybeFrontLeftResult.isPresent()) {
      tagResult.add(maybeFrontLeftResult.orElseThrow());
    }
  }

  public List<TagResult> getTagResult() {
    return tagResult;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  public void setCurrentScoringPipe(ReefPipe reefPipe) {
    this.reefPipe = reefPipe;
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.TAGS);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        backTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        frontRightLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        frontLeftLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
      }
      case CLOSEST_REEF_TAG_CLOSEUP -> {
        backTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        frontRightLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        frontLeftLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
      }
      case STATION_TAGS -> {
        backTagLimelight.setState(LimelightState.STATION_TAGS);
        frontRightLimelight.setState(LimelightState.STATION_TAGS);
        frontLeftLimelight.setState(LimelightState.STATION_TAGS);
      }
      case CORAL_DETECTION -> {
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.TAGS);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
      case ALGAE_DETECTION -> {
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.ALGAE);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    return frontRightLimelight.getAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    backTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontRightLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontLeftLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setClosestScoringReefAndPipe(int tagID, ReefPipe currentScoringPipe) {
    reefPipe = currentScoringPipe;
    frontRightLimelight.setClosestScoringReefTag(tagID);
    frontLeftLimelight.setClosestScoringReefTag(tagID);
    backTagLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || frontRightLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || frontLeftLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyScoringTagLimelightOnline() {
    if ((frontLeftLimelight.getState() == LimelightState.TAGS
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (frontLeftLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontLeftLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontRightLimelight.getState() == LimelightState.TAGS
            || frontRightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || frontRightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (frontRightLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontRightLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((backTagLimelight.getState() == LimelightState.TAGS
            || backTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || backTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (backTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || backTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontLeftLimelight.getState() == LimelightState.TAGS
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (frontLeftLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontLeftLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontRightLimelight.getState() == LimelightState.TAGS
            || frontRightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || frontRightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (frontRightLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontRightLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }

  public void updateDistanceFromReef(double distanceFromReef) {
    DogLog.log("Vision/DistanceFromReef", distanceFromReef);
    if (getState() == VisionState.CLOSEST_REEF_TAG) {
      if (distanceFromReef < REEF_CLOSEUP_DISTANCE) {
        setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
      }
    }
  }

  public Optional<Pose2d> getLollipopPose(LocalizationSubsystem localization) {
    // TODO: Update for new camera setup
    return Optional.empty();
    // var maybeAlgaeResult = frontCoralLimelight.getAlgaeResult();

    // if (maybeAlgaeResult.isEmpty()) {
    //   return Optional.empty();
    // }

    // var algaeResult = maybeAlgaeResult.orElseThrow();
    // var angleToCoral =
    //     GamePieceDetectionUtil.getFieldRelativeAngleToGamePiece(
    //         localization.getPose(algaeResult.timestamp()), algaeResult);

    // return Optional.of(
    //     new Pose2d(
    //         GamePieceDetectionUtil.calculateFieldRelativeLollipopTranslationFromCamera(
    //             localization.getPose(algaeResult.timestamp()), algaeResult),
    //         Rotation2d.fromDegrees(angleToCoral)));
  }
}
