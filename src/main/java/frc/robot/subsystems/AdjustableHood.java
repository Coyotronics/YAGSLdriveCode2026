// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Adjustable hood subsystem using 2 NEO Vortex motors (SparkFlex).
 * The follower motor is loosely coupled — it receives the same position
 * setpoint but reaches it using its own PID independently.
 */
public class AdjustableHood extends SubsystemBase {

    // ─── Motor controllers ─────────────────────────────────────────────
    private final SparkFlex leaderSpark;
    private final SparkFlex followerSpark;

    // ─── YAMS SmartMotorControllers ────────────────────────────────────
    private final SmartMotorController leaderMotor;
    private final SmartMotorController followerMotor;

    // ─── YAMS Arm mechanism ────────────────────────────────────────────
    private final Arm arm;

    public AdjustableHood() {
        // ── Vendor motor controller objects ──────────────────────────────
        leaderSpark = new SparkFlex(HoodConstants.HOOD_LEADER_ID, MotorType.kBrushless);
        followerSpark = new SparkFlex(HoodConstants.HOOD_FOLLOWER_ID, MotorType.kBrushless);

        // ── Follower SmartMotorController (must be created BEFORE leader) ─
        SmartMotorControllerConfig followerConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD,
                        HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION)
                .withGearing(HoodConstants.GEAR_RATIO)
                .withSoftLimit(HoodConstants.SOFT_LIMIT_LOWER, HoodConstants.SOFT_LIMIT_UPPER)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
                .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("HoodFollower", TelemetryVerbosity.HIGH);

        followerMotor = new SparkWrapper(followerSpark, DCMotor.getNeoVortex(1), followerConfig);

        // ── Leader SmartMotorController ─────────────────────────────────
        SmartMotorControllerConfig leaderConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD,
                        HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION)
                .withGearing(HoodConstants.GEAR_RATIO)
                .withSoftLimit(HoodConstants.SOFT_LIMIT_LOWER, HoodConstants.SOFT_LIMIT_UPPER)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
                .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("HoodLeader", TelemetryVerbosity.HIGH)
                .withLooselyCoupledFollowers(followerMotor);

        leaderMotor = new SparkWrapper(leaderSpark, DCMotor.getNeoVortex(1), leaderConfig);

        // ── YAMS Arm (wraps the leader motor) ───────────────────────────
        arm = new Arm(
                new ArmConfig(leaderMotor)
                        .withLength(HoodConstants.ARM_LENGTH)
                        .withMass(HoodConstants.ARM_MASS)
                        .withHardLimit(HoodConstants.HARD_LIMIT_LOWER, HoodConstants.HARD_LIMIT_UPPER)
                        .withStartingPosition(HoodConstants.STARTING_POSITION)
                        .withTelemetry("AdjustableHood", TelemetryVerbosity.HIGH));
    }

    // ─── Periodic methods ───────────────────────────────────────────────

    @Override
    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

    // ─── Public API ─────────────────────────────────────────────────────

    /**
     * Command that drives the hood to the requested angle and holds it.
     *
     * @param angle target hood angle
     * @return a {@link Command} that holds the hood at the given angle
     */
    public Command setHoodAngle(Angle angle) {
        return arm.setAngle(angle);
    }

    /**
     * Command that drives the hood to the requested angle and finishes
     * once the hood is within the given tolerance.
     *
     * @param angle     target hood angle
     * @param tolerance acceptable error
     * @return a {@link Command} that ends when the hood reaches the target
     */
    public Command setHoodAngleTo(Angle angle, Angle tolerance) {
        return arm.runTo(angle, tolerance);
    }

    /**
     * @return the current hood angle
     */
    public Angle getHoodAngle() {
        return arm.getAngle();
    }

    /**
     * Trigger that fires when the hood is within {@code tolerance} of
     * {@code angle}.
     */
    public Trigger isAtAngle(Angle angle, Angle tolerance) {
        return arm.isNear(angle, tolerance);
    }
}
