package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterFlywheel extends SubsystemBase {

  // Top flywheel
  private final SparkMax topFlywheelMotor = new SparkMax(13, MotorType.kBrushless); //left
  private final SparkMax topFlywheelFollowerMotor = new SparkMax(60, MotorType.kBrushless); //right

  private final SmartMotorControllerConfig topMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(0.3447, 0, 0.0025)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("TopFlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(60))
          .withMotorInverted(true)
          .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withFollowers(Pair.of(topFlywheelFollowerMotor, true))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController topMotor =
      new SparkWrapper(topFlywheelMotor, DCMotor.getNeoVortex(2), topMotorConfig);

  private final FlyWheelConfig topFlywheelConfig =
      new FlyWheelConfig(topMotor)
          .withDiameter(Inches.of(3.93701))
          .withMass(Pounds.of(0.4516))
          .withTelemetry("TopFlywheel", TelemetryVerbosity.HIGH);

  private final FlyWheel topFlywheel = new FlyWheel(topFlywheelConfig);

  // Bottom flywheel
  private final SparkMax bottomFlywheelMotor = new SparkMax(46, MotorType.kBrushless); //right
  private final SparkMax bottomFlywheelFollowerMotor = new SparkMax(39, MotorType.kBrushless); //left

  private final SmartMotorControllerConfig bottomMotorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(0.3447, 0, 0.0025)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("BottomFlywheelMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(60))
          .withMotorInverted(true)
          .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01))
          .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
          .withFollowers(Pair.of(bottomFlywheelFollowerMotor, true))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController bottomMotor =
      new SparkWrapper(bottomFlywheelMotor, DCMotor.getNeoVortex(2), bottomMotorConfig);

  private final FlyWheelConfig bottomFlywheelConfig =
      new FlyWheelConfig(bottomMotor)
          .withDiameter(Inches.of(3.93701))
          .withMass(Pounds.of(0.4516))
          .withTelemetry("BottomFlywheel", TelemetryVerbosity.HIGH);

  private final FlyWheel bottomFlywheel = new FlyWheel(bottomFlywheelConfig);

  public ShooterFlywheel() {}

  public AngularVelocity getTopRPM() {
    return topFlywheel.getSpeed();
  }

  public AngularVelocity getBottomRPM() {
    return bottomFlywheel.getSpeed();
  }

  public Command setBothVelocityCommand(AngularVelocity velocity) {
    return run(() -> {
        topFlywheel.setMechanismVelocitySetpoint(velocity);
        bottomFlywheel.setMechanismVelocitySetpoint(velocity);
    });
}

  public void setBothVelocitySetpoint(AngularVelocity velocity) {
    topFlywheel.setMechanismVelocitySetpoint(velocity);
    bottomFlywheel.setMechanismVelocitySetpoint(velocity);
  }

  public Command setBothDutyCycleCommand(double dutyCycle) {
    return run(() -> {
        topFlywheel.set(dutyCycle);
        bottomFlywheel.set(dutyCycle);
    });
}

  public void setBothDutyCycleSetpoint(double dutyCycle) {
    topFlywheel.setDutyCycleSetpoint(dutyCycle);
    bottomFlywheel.setDutyCycleSetpoint(dutyCycle);
  }

  public Command stopCommand() {
    return topFlywheel.set(0).alongWith(bottomFlywheel.set(0));
  }

  @Override
  public void periodic() {
    topFlywheel.updateTelemetry();
    bottomFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    topFlywheel.simIterate();
    bottomFlywheel.simIterate();
  }

public AngularVelocity getSpeed() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSpeed'");
}
}