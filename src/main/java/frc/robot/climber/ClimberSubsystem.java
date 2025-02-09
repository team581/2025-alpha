package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private static final double TOLERANCE = 1;
  private final TalonFX motor;
  private final CANcoder encoder;
  private double currentAngle;
  private TempClimberState tempState = TempClimberState.STOPPED;

  public ClimberSubsystem(TalonFX motor, CANcoder encoder) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    // motor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(70)
                        .withSupplyCurrentLimit(50))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());

    this.motor = motor;
    this.encoder = encoder;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (tempState) {
      case STOPPED -> motor.disable();
      // case UP -> motor.setVoltage(8);
      // case DOWN -> motor.setVoltage(-8);
      case UP -> motor.disable();
      case DOWN -> motor.disable();
    }

    DogLog.log("Climber/TempState", tempState);
    DogLog.log("Climber/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Climber/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Climber/OutputVoltage", motor.getMotorVoltage().getValueAsDouble());
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  public void setState(TempClimberState newState) {
    tempState = newState;
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble());
  }

  public boolean atGoal() {
    return MathUtil.isNear(getState().angle, currentAngle, TOLERANCE);
  }
}
