package org.firstinspires.ftc.teamcode.HyperionRobotics.viper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * 4-stage Viper slide controlled by encoder setpoints (stage 0–4).
 */
public class ViperArm {
    public enum Stage {
        STOWED(RobotConstants.VIPER_STAGE_0),
        LOW(RobotConstants.VIPER_STAGE_1),
        MID(RobotConstants.VIPER_STAGE_2),
        HIGH(RobotConstants.VIPER_STAGE_3),
        MAX(RobotConstants.VIPER_STAGE_4);

        public final int ticks;

        Stage(int ticks) {
            this.ticks = ticks;
        }
    }

    private final DcMotorEx primary;
    private final DcMotorEx secondary; // null if single-motor
    private Stage currentStage = Stage.STOWED;

    public ViperArm(HardwareMap hardwareMap) {
        primary = safeGet(hardwareMap, RobotConstants.VIPER_MOTOR);
        if (RobotConstants.VIPER_DUAL_MOTOR) {
            secondary = safeGet(hardwareMap, RobotConstants.VIPER_MOTOR_2);
        } else {
            secondary = null;
        }

        if (primary != null) {
            configureMotor(primary);
            primary.setDirection(DcMotor.Direction.REVERSE);
        }
        
        if (secondary != null) {
            configureMotor(secondary);
            secondary.setDirection(DcMotor.Direction.REVERSE);
        }

        if (primary != null) {
            setStage(Stage.STOWED);
        }
    }

    private DcMotorEx safeGet(HardwareMap hw, String name) {
        try {
            return hw.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
    }

    public void setStage(Stage stage) {
        currentStage = stage;
        goToTicks(stage.ticks);
    }

    public void goToTicks(int ticks) {
        if (primary == null) return;
        int clipped = Range.clip(ticks, 0, RobotConstants.VIPER_MAX_TICKS);
        primary.setTargetPosition(clipped);
        primary.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        primary.setPower(RobotConstants.VIPER_POWER);
        if (secondary != null) {
            secondary.setTargetPosition(clipped);
            secondary.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            secondary.setPower(RobotConstants.VIPER_POWER);
        }
    }

    /** Manual jog while holding a bumper; switches to open-loop briefly. */
    public void jog(double power) {
        if (primary == null) return;
        if (Math.abs(power) < 0.05) {
            holdPosition();
            return;
        }
        int next = Range.clip(
                primary.getCurrentPosition() + (int) (power * 90),
                0,
                RobotConstants.VIPER_MAX_TICKS);
        goToTicks(next);
    }

    public void holdPosition() {
        if (primary == null) return;
        goToTicks(primary.getCurrentPosition());
    }

    public void stop() {
        if (primary != null) primary.setPower(0.0);
        if (secondary != null) {
            secondary.setPower(0.0);
        }
    }

    public boolean isAtTarget() {
        if (primary == null) return true;
        return Math.abs(primary.getCurrentPosition() - primary.getTargetPosition())
                <= RobotConstants.VIPER_TOLERANCE;
    }

    public int getCurrentTicks() {
        return (primary != null) ? primary.getCurrentPosition() : 0;
    }

    public Stage getCurrentStage() {
        return currentStage;
    }

    public void nextStage() {
        Stage[] stages = Stage.values();
        int idx = Math.min(currentStage.ordinal() + 1, stages.length - 1);
        setStage(stages[idx]);
    }

    public void previousStage() {
        Stage[] stages = Stage.values();
        int idx = Math.max(currentStage.ordinal() - 1, 0);
        setStage(stages[idx]);
    }

    public int getTargetTicks() {
        return (primary != null) ? primary.getTargetPosition() : 0;
    }
    public int getPositionError() {
        return getTargetTicks() - getCurrentTicks();
    }
}
