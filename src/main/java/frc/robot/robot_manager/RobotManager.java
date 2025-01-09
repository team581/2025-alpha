package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RobotManager extends StateMachine<RobotState> {
  private final SwerveSubsystem swerve;

  public RobotManager(SwerveSubsystem swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);

    this.swerve = swerve;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_ALGAE,
              IDLE_CORAL,
              IDLE_BOTH_GP,
              PROCESSOR_WAITING,
              NET_WAITING,
              CORAL_L1_WAITING,
              CORAL_L2_WAITING,
              CORAL_L3_WAITING,
              CORAL_L4_WAITING,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING ->
          currentState;

        //   TODO: add check for PREPARE_TO_SCORE transitions
      case PROCESSOR_PREPARE_TO_SCORE -> true ? RobotState.PROCESSOR_SCORING : currentState;

      case NET_PREPARE_TO_SCORE -> true ? RobotState.NET_SCORING : currentState;

      case CORAL_L1_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L1_SCORING : currentState;

      case CORAL_L2_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L2_SCORING : currentState;

      case CORAL_L3_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L3_SCORING : currentState;

      case CORAL_L4_PREPARE_TO_SCORE -> true ? RobotState.CORAL_L4_SCORING : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // TODO: Implement
    switch (newState) {
      case SCORE_ASSIST -> {
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.SCORE_ASSIST);
        } else {
          swerve.setState(SwerveState.SCORE_ASSIST);
        }
      }
      case PURPLE_ALGIN -> {
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.PURPLE_ALIGN);
        } else {
          swerve.setState(SwerveState.PURPLE_ALIGN);
        }
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Continuous state actions
    switch (getState()) {
      default -> {}
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_BOTH_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }
}
