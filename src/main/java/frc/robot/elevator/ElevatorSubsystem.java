package frc.robot.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState> {
  private static double rotationsToInches(double rotations) {
    return Units.degreesToRotations(rotations)
        * (RobotConfig.get().elevator().rotationsToDistance());
  }

  private static double inchesToRotations(double inches) {
    return Units.rotationsToDegrees(inches / (RobotConfig.get().elevator().rotationsToDistance()));
  }

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final PositionVoltage positionRequest = new PositionVoltage(ElevatorState.STOWED.value);
  private double goalHeight = ElevatorState.STOWED.value;

  // Homing
  private boolean preMatchHomingOccured = false;
  private double lowestSeenHeight = 0.0;
  private double height;
  private HomingState homingState = HomingState.PRE_MATCH_HOMING;

  public ElevatorSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.STOWED);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  @Override
  public void disabledPeriodic() {
    double currentHeight = height;

    if (currentHeight < lowestSeenHeight) {
      lowestSeenHeight = currentHeight;
    }
  }

  @Override
  public void robotPeriodic() {
    switch (homingState) {
      case NOT_HOMED:
        topMotor.stopMotor();
        bottomMotor.stopMotor();
        break;
      case PRE_MATCH_HOMING:
        topMotor.stopMotor();
        bottomMotor.stopMotor();

        if (DriverStation.isDisabled()) {
          // Wait until enable to do homing code
        } else {

          if (!preMatchHomingOccured) {
            double homingEndPosition = RobotConfig.get().elevator().homingEndPosition();
            double homedPosition = homingEndPosition + (height - lowestSeenHeight);
            topMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));
            bottomMotor.setPosition(Units.degreesToRotations(inchesToRotations(homedPosition)));

            preMatchHomingOccured = true;
            homingState = HomingState.HOMED;
          }
        }
        break;
      case HOMED:
        {
          double setHeight = clampHeight(goalHeight);

          if (MathUtil.isNear(0, height, RobotConfig.get().elevator().tolerance())) {
            topMotor.disable();
            bottomMotor.disable();
          } else {
            //move this to collectinputs
            topMotor.setControl(
                positionRequest.withPosition(
                    Units.degreesToRotations(inchesToRotations(setHeight))));
            bottomMotor.setControl(
                positionRequest.withPosition(
                    Units.degreesToRotations(inchesToRotations(setHeight))));
          }

          break;
        }
      case MID_MATCH_HOMING:
        throw new IllegalStateException("Elevator can't do mid match homing");
    }

    DogLog.log("Elevator/Height", height);
    DogLog.log("Elevator/GoalHeight", goalHeight);
  }

  public boolean atGoal(ElevatorState elevatorState) {
   return MathUtil.isNear(goalHeight, height, RobotConfig.get().elevator().tolerance());
 
  }

  @Override
  protected void collectInputs() {
    // Calculate average height of the two motors
    height =
        (rotationsToInches(Units.rotationsToDegrees(topMotor.getPosition().getValueAsDouble()))
                + rotationsToInches(Units.rotationsToDegrees(bottomMotor.getPosition().getValueAsDouble())))
            / 2.0;
  }
  public HomingState getHomingState() {
    return homingState;
  }
}
