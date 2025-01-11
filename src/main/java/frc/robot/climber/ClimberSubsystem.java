package frc.robot.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
    private final TalonFX motor;

    public ClimberSubsystem(TalonFX motor) {
        super(SubsystemPriority.CLIMBER, ClimberState.HOMING);
        this.motor = motor;
    }
}
