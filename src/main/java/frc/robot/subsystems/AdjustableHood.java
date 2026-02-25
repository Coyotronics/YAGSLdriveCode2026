package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorFactory;

public class AdjustableHood extends SubsystemBase {

        private final Pivot pivot;

        public AdjustableHood() {
                SmartMotorControllerConfig followerConfig = new SmartMotorControllerConfig(this)
                                .withGearing(HoodConstants.GEAR_RATIO)
                                .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
                                .withIdleMode(MotorMode.BRAKE)
                                .withStatorCurrentLimit(Amps.of(HoodConstants.CURRENT_LIMIT))
                                .withSoftLimit(Degrees.of(HoodConstants.MIN_ANGLE_DEG),
                                                Degrees.of(HoodConstants.MAX_ANGLE_DEG))
                                .withMomentOfInertia(Inches.of(HoodConstants.ARM_LENGTH_INCHES),
                                                Pounds.of(HoodConstants.ARM_MASS_LBS))
                                .withTelemetry("Hood/Follower", TelemetryVerbosity.LOW);

                SmartMotorController followerSMC = SmartMotorFactory.create(
                                new SparkFlex(HoodConstants.FOLLOWER_MOTOR_ID, MotorType.kBrushless),
                                DCMotor.getNeoVortex(1),
                                followerConfig).orElseThrow();

                SmartMotorControllerConfig leaderConfig = new SmartMotorControllerConfig(this)
                                .withGearing(HoodConstants.GEAR_RATIO)
                                .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
                                .withIdleMode(MotorMode.BRAKE)
                                .withStatorCurrentLimit(Amps.of(HoodConstants.CURRENT_LIMIT))
                                .withSoftLimit(Degrees.of(HoodConstants.MIN_ANGLE_DEG),
                                                Degrees.of(HoodConstants.MAX_ANGLE_DEG))
                                .withClosedLoopTolerance(Degrees.of(HoodConstants.TOLERANCE_DEG))
                                .withMomentOfInertia(Inches.of(HoodConstants.ARM_LENGTH_INCHES),
                                                Pounds.of(HoodConstants.ARM_MASS_LBS))
                                .withLooselyCoupledFollowers(followerSMC)
                                .withTelemetry("Hood/Leader", TelemetryVerbosity.HIGH);

                SmartMotorController leaderSMC = SmartMotorFactory.create(
                                new SparkFlex(HoodConstants.LEADER_MOTOR_ID, MotorType.kBrushless),
                                DCMotor.getNeoVortex(1),
                                leaderConfig).orElseThrow();

                PivotConfig pivotConfig = new PivotConfig(leaderSMC)
                                .withHardLimit(Degrees.of(HoodConstants.MIN_ANGLE_DEG),
                                                Degrees.of(HoodConstants.MAX_ANGLE_DEG))
                                .withMOI(Inches.of(HoodConstants.ARM_LENGTH_INCHES),
                                                Pounds.of(HoodConstants.ARM_MASS_LBS))
                                .withStartingPosition(Degrees.of(HoodConstants.MIN_ANGLE_DEG))
                                .withTelemetry("Hood", TelemetryVerbosity.HIGH);

                pivot = new Pivot(pivotConfig);
        }

        public Angle getAngle() {
                return pivot.getAngle();
        }

        public Command setAngle(Angle angle) {
                return pivot.setAngle(angle);
        }

        public Command runTo(Angle angle) {
                return pivot.runTo(angle, Degrees.of(HoodConstants.TOLERANCE_DEG));
        }

        public Command max() {
                return runTo(Degrees.of(HoodConstants.MAX_ANGLE_DEG));
        }

        public Command min() {
                return runTo(Degrees.of(HoodConstants.MIN_ANGLE_DEG));
        }

        public Trigger isNear(Angle angle, Angle tolerance) {
                return pivot.isNear(angle, tolerance);
        }

        @Override
        public void periodic() {
                pivot.updateTelemetry();
        }

        @Override
        public void simulationPeriodic() {
                pivot.simIterate();
        }
}
