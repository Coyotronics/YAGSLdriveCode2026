package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

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

public class IntakeRollers extends SubsystemBase {

  // same exact logic, just renamed for an intake roller

    private final SparkMax intakeRollerMotor = new SparkMax(12, MotorType.kBrushless); // TODO: SET CAN IDS


    private final SmartMotorControllerConfig intakeRollerMotorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0.3447, 0, 0.0025)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(30))
                    .withMotorInverted(true)
                    .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01))
                    .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                    .withControlMode(ControlMode.CLOSED_LOOP);
                    //.withVoltageCompensation(Volts.of(12));

    private final SmartMotorController intakeRollerMotorController =
            new SparkWrapper(intakeRollerMotor, DCMotor.getNeoVortex(1), intakeRollerMotorConfig);

    private final FlyWheelConfig intakeRollerConfig =
            new FlyWheelConfig(intakeRollerMotorController)
                    .withDiameter(Inches.of(3.93701))
                    .withMass(Pounds.of(0.4516))
                    .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    private final FlyWheel intakeRoller = new FlyWheel(intakeRollerConfig);

    public IntakeRollers() {
    }

    public AngularVelocity getRPM() {
        return intakeRoller.getSpeed();
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return intakeRoller.setSpeed(velocity);
    }

    public void setVelocitySetpoint(AngularVelocity velocity) {
        intakeRoller.setMechanismVelocitySetpoint(velocity);
    }

    public Command setDutyCycle(double dutyCycle) {
        return intakeRoller.set(dutyCycle);
    }

    public Command stopCommand() {
        return intakeRoller.set(0);
    }

    public void periodic() {
        intakeRoller.updateTelemetry();
    }

    public void simulationPeriodic() {
        intakeRoller.simIterate();
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
         intakeRoller.setDutyCycleSetpoint(dutyCycle);
    }
}