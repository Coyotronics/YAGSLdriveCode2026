package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.opencv.features2d.FlannBasedMatcher;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class IntakeArm extends SubsystemBase
{

  private SparkMax             m_masterMotor   = new SparkMax(2, MotorType.kBrushless); // TODO: fix this ID, something is wrong

  //private SparkAbsoluteEncoder m_masterAbsoluteEncoder = m_masterMotor.getAbsoluteEncoder();
 

  private SmartMotorControllerConfig masterConfig            = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)

      .withClosedLoopController(3.5, 0, 0.3)
      .withFeedforward(new ArmFeedforward(0.15, 0.5, 0, 0))

      .withSimClosedLoopController(3.5, 0, 0.3)
      .withSimFeedforward(new ArmFeedforward(0.15, 0.5, 0, 0))
      .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(25, 1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))      
      //.withExternalEncoder(m_masterAbsoluteEncoder)
      // .withSoftLimit(Degrees.of(-5), Degrees.of(120))
      //.withExternalEncoderInverted(true)
      // .withExternalEncoderZeroOffset(masterAbsoluteEncoderZeroOffset) // Remove if configured in REV HW Client
     // .withUseExternalFeedbackEncoder(true)
      
      .withResetPreviousConfig(false);

  private SmartMotorController       masterMotorController   = new SparkWrapper(m_masterMotor, DCMotor.getNeoVortex(1),
                                                                                masterConfig);


  private ArmConfig armCfg = new ArmConfig(masterMotorController)
      // Hard limit is applied to the simulation.
     // .withHardLimit(Degrees.of(-10), Degrees.of(130))
      // Length and mass of your arm for sim.
      .withLength(Inches.of(15))
      .withMass(Pounds.of(9))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
      .withStartingPosition(Degrees.zero());


  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  public IntakeArm()
  {
  }
  public Command setDegreeCommand(double degree) {
    return arm.setAngle(Degrees.of(degree));
  }

  public void setVelocity(double velocity){
    arm.setMechanismVelocitySetpoint(DegreesPerSecond.of(velocity));
  }

  public void setAngleSetpoint(Angle degree) {
      arm.setMechanismPositionSetpoint(degree);
  }

  public Angle getAngle() {
    return arm.getAngle();
  }

  public void setDutyCycleSetpoint(double dutyCycle) {
    arm.setDutyCycleSetpoint(dutyCycle);
  } 

  public void set(double dutyCycle) {
    arm.set(dutyCycle);
  } 

  public Command setDutyCycle(double dutyCycle) {
    return arm.set(dutyCycle);
  }

  public Command stopCommand(){
    return arm.set(0);
  }
  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

  public void changeDegreeCommand(double degree) {
    arm.setAngle(arm.getAngle().plus(Degrees.of(degree)));
  }



  //pop out command
  public final Current CurrentThreshold = Amps.of(10); // Current threshold to detect when the arm has hit its hard limit 
	Debouncer currentDebouncer = new Debouncer(0.001); // Current threshold is only detected if exceeded for 0.1
	Voltage runVolts = Volts.of(4); // Volts required to run the mechanism down. Could be negative if the mechanism
//() -> currentDebouncer.calculate(masterMotorController.getStatorCurrent().gte(CurrentThreshold))

public Command popOut(double volts) {
		return Commands.startRun(masterMotorController::stopClosedLoopController, // Stop the closed loop controller
				() -> {
              masterMotorController.setDutyCycle(volts);
              System.out.println("10 fucking volts!!");
               }) // Set the voltage of the motor
				.until(() -> false)
        
				.finallyDo(() -> {
					masterMotorController.setVoltage(Volts.of(0)); // Stop the motor
					masterMotorController.setEncoderPosition(Degrees.of(0));
					masterMotorController.startClosedLoopController();
				});
	}


}