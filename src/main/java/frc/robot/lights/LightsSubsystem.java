package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;

  public LightsSubsystem(CANdle candle) {
      super(SubsystemPriority.LIGHTS, LightsState.READY);

    this.candle = candle;
  }

  public void setState(LightsState newState) {
    setStateFromRequest(newState);
  }
}
