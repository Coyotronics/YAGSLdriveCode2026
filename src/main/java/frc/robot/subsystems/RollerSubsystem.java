package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RollerSubsystem extends SubsystemBase {

  //me when I lowkirkenuinely say "a whole conveyor system is just a flywheel with a belt on it": https://www.youtube.com/watch?v=9n2j8qQYp3o

    private final SparkMax flywheelMotor = new SparkMax(23, MotorType.kBrushless); //TODO: SET  CAN IDS
    private final SparkMax flywheelFollowerMotor = new SparkMax(7, MotorType.kBrushless); //TODO: SET  CAN IDS
    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0.3447, 0, 0.0025)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(60))//For flywheel, stator current lm can be 60A/80A //talon fx can handle up to 260A
                    .withMotorInverted(true)
                    .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01)) //thanks 3561!
                    .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                    .withFollowers(Pair.of(flywheelFollowerMotor, true))
                    .withControlMode(ControlMode.CLOSED_LOOP);
                    //.withVoltageCompensation(Volts.of(12));

    private final SmartMotorController motor =
            new SparkWrapper(flywheelMotor, DCMotor.getNeoVortex(2), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(Inches.of(3.93701))
                    .withMass(Pounds.of(0.4516))
                    .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

    private final FlyWheel conevyor = new FlyWheel(flywheelConfig);

    public RollerSubsystem() {
    }

    public AngularVelocity getRPM() {
        return conevyor.getSpeed();
    }

    public Command setVelocityommand(AngularVelocity velocity) {
        return conevyor.setSpeed(velocity);
    }

    public void setVelocitySetpoint(AngularVelocity velocity)
    {
        conevyor.setMechanismVelocitySetpoint(velocity);
    }

    public Command setDutyCycle(double dutyCycle) {
        return conevyor.set(dutyCycle);
    }

    public Command stopCommand() {
        return conevyor.set(0);
    }

    public void periodic() {
        conevyor.updateTelemetry();
    }

    public void simulationPeriodic() {
        conevyor.simIterate();
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
         conevyor.setDutyCycleSetpoint(dutyCycle);
    }



}