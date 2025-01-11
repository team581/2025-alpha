package frc.robot.pivot;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class PivotSubsystem extends StateMachine<PivotState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorVelocity;

  private final IntakeSubsystem intake;

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(PivotState.STOWED.angle).withEnableFOC(false);

  public PivotSubsystem(TalonFX motor, IntakeSubsystem intake) {
    super(SubsystemPriority.PIVOT, PivotState.HOMING);

    motor.getConfigurator().apply(RobotConfig.get().pivot().motorConfig());

    this.motor = motor;
    this.intake = intake;
  }

  @Override
  protected void afterTransition(PivotState newState) {
    switch (newState) {
      case HOMING -> {
        motor.setVoltage(0);
      }
      case STOWED -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(PivotState.STOWED.angle)));
      }
      case CORAL_SCORE -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(getScoreDirection().angle)));
      }
      default -> {}
    }
  }

  @Override
  protected PivotState getNextState(PivotState currentState) {
    if (currentState == PivotState.HOMING
        && motorVelocity < RobotConfig.get().pivot().homingVelocityThreshold()) {
      motor.setPosition(RobotConfig.get().pivot().homingPosition());
      return PivotState.STOWED;
    }

    // Don't do anything
    return currentState;
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorVelocity = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  private PivotState getScoreDirection() {
    if (!intake.getRightSensor()) {
      return PivotState.CORAL_LEFT;
    }

    return PivotState.CORAL_RIGHT;
  }

  public void setState(PivotState newState) {
    if (getState() != PivotState.HOMING) {

      setStateFromRequest(newState);
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case HOMING -> false;
      default -> MathUtil.isNear(getState().angle, motorAngle, 1);
    };
  }
}
