package frc.robot.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private final ClimberConfig CONFIG = RobotConfig.get().climber();
  private final TalonFX motor;
  private final CANcoder encoder;
  // all in inches

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.UNHOMED);

    motor.getConfigurator().apply(CONFIG.motorConfig());
    encoder.getConfigurator().apply(CONFIG.CANcoderConfig());

    this.motor = motor;
    this.encoder = encoder;
  }

  @Override
  protected ClimberState getNextState(ClimberState currentState) {
    if (currentState == ClimberState.HOMING && motor.getStatorCurrent().getValueAsDouble() > RobotConfig.get().climber().homingCurrentThreshold()) {
      motor.setPosition(RobotConfig.get().climber().homingPosition());
      return ClimberState.STOWED;
    }

    // Do nothing
    return currentState;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case UNHOMED -> {
        motor.disable();
      }
      case HOMING -> {
        //TODO: Set homing voltage to like 2ish
        motor.setVoltage(0);
      }
      case STOWED -> {
          motor.disable();
      }
      case LINEUP, HANGING -> {
        if (currentPositionLessThanGoal()) {
          motor.setVoltage(0.1);
        } else {
          motor.setVoltage(-0.1);
        }
      }
    }
  }

  public boolean currentPositionLessThanGoal() {
    return getPosition() < getState().height;
  }

  public double getPosition() {
    return encoder.getPosition().getValueAsDouble() * CONFIG.rotationsToInches();
  }
}
