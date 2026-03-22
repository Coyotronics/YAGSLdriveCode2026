package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public class HoodSubsystem extends SubsystemBase {//Modeled as a pivot, since it's not really affected by gravity

  private final SparkMax hoodMotor = new SparkMax(47, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(4,0,0)
              //DegreesPerSecond.of(180),  
              //DegreesPerSecondPerSecond.of(90))  //Don't add it unless it's not increasing fast enough
          .withSimClosedLoopController(4,0,0)
              //DegreesPerSecond.of(180),
              //DegreesPerSecondPerSecond.of(90))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(265, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withVoltageCompensation(Volts.of(12))
          .withMotorInverted(false)
          .withStartingPosition(Degrees.of(0))
          //.withClosedLoopRampRate(Seconds.of(0.25)) //Don't add it unless it's too fast(limit the rate & help slow it down)->check yams docs
          //.withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.2, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0,0,0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new SparkWrapper(hoodMotor, DCMotor.getNeoVortex(1), motorConfig);

 /*  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), // back from robot center
                                                  Meters.of(0),     // centered left/right
                                                  Meters.of(0.5))); // up from the floor reference*/

  private final PivotConfig m_config =
      new PivotConfig(motor)
          .withHardLimit(Degrees.of(0), Degrees.of(30))
          .withSoftLimits(Degrees.of(-10), Degrees.of(27))
          .withTelemetry("Hood", TelemetryVerbosity.HIGH)
          .withStartingPosition(Degrees.of(0))
          //.withMOI(Inches.of(7), Pounds.of(2))
          .withMOI( KilogramSquareMeters.of(Pounds.of(8).in(Kilograms) * Inches.of(14).in(Meters) * Inches.of(14).in(Meters)));

  private final Pivot hood = new Pivot(m_config);

  public HoodSubsystem() {}

  public Command setDegreeCommand(double degree) {
    return hood.setAngle(Degrees.of(degree));
  }

  public void setVelocity(double velocity){
    hood.setMechanismVelocitySetpoint(DegreesPerSecond.of(velocity));
  }

  public void setAngleSetpoint(Angle degree) {
      hood.setMechanismPositionSetpoint(degree);
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public void setDutyCycleSetpoint(double dutyCycle) {
    hood.setDutyCycleSetpoint(dutyCycle);
  } 

  public void set(double dutyCycle) {
    hood.set(dutyCycle);
  } 

  public Command setDutyCycle(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command stopCommand(){
    return hood.set(0);
  }
  @Override
  public void periodic() {
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }

  public void changeDegreeCommand(double degree) {
    hood.setAngle(hood.getAngle().plus(Degrees.of(degree)));
  }


}
//Live tuning, so we don't really need sysId
//public Command sysId() {
  //   return hood.sysId(
  //       Volts.of(4.0), // maximumVoltage
  //       Volts.per(Second).of(0.5), // step
  //       Seconds.of(8.0) // duration
  //       );
  // }