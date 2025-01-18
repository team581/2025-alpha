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
  private final double tolerance = 1.0;

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.UNHOMED);

    motor.getConfigurator().apply(CONFIG.motorConfig());
    encoder.getConfigurator().apply(CONFIG.CANcoderConfig());

    this.motor = motor;
    this.encoder = encoder;
  }

  @Override
  protected ClimberState getNextState(ClimberState currentState) {
    return switch (currentState) {
      case UNHOMED, STOWED, LINEUP, HANGING -> currentState;
      case HOMING ->
          encoder.getVelocity().getValueAsDouble() < CONFIG.homingVelocityThreshold()
                  && timeout(0.1)
              ? ClimberState.STOWED
              : currentState;
    };
  }

  @Override
  protected void afterTransition(ClimberState newState) {
    switch (newState) {
      case UNHOMED -> motor.disable();
      case HOMING -> motor.setVoltage(CONFIG.homingVoltage());
      case STOWED -> {
        if (atGoal()) {
          motor.disable();
        } else {
          motor.setVoltage(0.0);
        }
      }
      case LINEUP -> {
        if (atGoal()) {
          motor.disable();
        } else {
          motor.setVoltage(0.0);
        }
      }
      case HANGING -> {
        if (atGoal()) {
          motor.disable();
        } else {
          motor.setVoltage(0.0);
        }
      }
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> true;
      case STOWED, LINEUP, HANGING -> Math.abs(getPosition() - getState().height) < tolerance;
    };
  }

  public double getPosition() {
    return encoder.getPosition().getValueAsDouble() * CONFIG.rotationsToInches();
  }
}
