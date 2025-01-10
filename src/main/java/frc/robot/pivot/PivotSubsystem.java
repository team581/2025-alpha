package frc.robot.pivot;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class PivotSubsystem extends LifecycleSubsystem{
    private final TalonFX motor;

    public PivotSubsystem(TalonFX motor) {
        super(SubsystemPriority.PIVOT);

        this.motor = motor;
    }
}

// stowed state
// homing state
// rotate to see coral state
