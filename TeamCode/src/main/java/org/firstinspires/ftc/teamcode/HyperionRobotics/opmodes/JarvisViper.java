package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * Viper Slide ONLY Test
 *
 * No:
 *  - HyperionRobot
 *  - Drivetrain
 *  - Intake
 *  - Pinpoint
 *  - Limelight
 *
 * Controls:
 *
 * MANUAL
 *   Left Bumper  = UP
 *   Left Trigger = DOWN
 *
 * PRESETS
 *   D-Pad Up     = LOW
 *   D-Pad Right  = MID
 *   D-Pad Down   = HIGH
 *   D-Pad Left   = MAX
 *   B / Circle   = STOWED
 */

@TeleOp(
        name = "Jarvis Viper Test",
        group = "Test"
)
public class JarvisViper extends LinearOpMode {

    private ViperArm viper;

    @Override
    public void runOpMode() {

        // ================================================================
        // INITIALIZE ONLY THE VIPER
        // ================================================================

        viper = new ViperArm(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("================================");
        telemetry.addLine("       VIPER SLIDE TEST");
        telemetry.addLine("================================");
        telemetry.addLine();

        telemetry.addLine("MANUAL:");
        telemetry.addLine("Left Bumper  = UP");
        telemetry.addLine("Left Trigger = DOWN");
        telemetry.addLine();

        telemetry.addLine("PRESETS:");
        telemetry.addLine("D-Pad Up     = LOW");
        telemetry.addLine("D-Pad Right  = MID");
        telemetry.addLine("D-Pad Down   = HIGH");
        telemetry.addLine("D-Pad Left   = MAX");
        telemetry.addLine("B / Circle   = STOWED");
        telemetry.addLine();

        telemetry.addData(
                "Initial Position",
                "%d ticks",
                viper.getCurrentTicks()
        );

        telemetry.addLine();
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            viper.stop();
            return;
        }

        while (opModeIsActive()) {

            // ============================================================
            // CONTROL STATE
            // ============================================================

            boolean manualControlActive = false;
            boolean presetRequested = false;

            String command = "HOLD";


            // ============================================================
            // 1. MANUAL CONTROL
            // ================================================================

            if (gamepad1.left_bumper) {

                viper.jog(1.0);

                manualControlActive = true;
                command = "MANUAL UP";

            } else if (gamepad1.left_trigger > 0.1) {

                viper.jog(-1.0);

                manualControlActive = true;
                command = "MANUAL DOWN";
            }


            // ============================================================
            // 2. PRESET CONTROL
            // ================================================================

            /*
             * Only process presets when manual control is NOT active.
             *
             * This prevents the D-pad from fighting a manual jog command
             * during the same loop.
             */
            if (!manualControlActive) {

                if (gamepad1.dpad_up) {

                    viper.setStage(
                            ViperArm.Stage.LOW
                    );

                    presetRequested = true;
                    command = "PRESET LOW";

                } else if (gamepad1.dpad_right) {

                    viper.setStage(
                            ViperArm.Stage.MID
                    );

                    presetRequested = true;
                    command = "PRESET MID";

                } else if (gamepad1.dpad_down) {

                    viper.setStage(
                            ViperArm.Stage.HIGH
                    );

                    presetRequested = true;
                    command = "PRESET HIGH";

                } else if (gamepad1.dpad_left) {

                    viper.setStage(
                            ViperArm.Stage.MAX
                    );

                    presetRequested = true;
                    command = "PRESET MAX";

                } else if (gamepad1.circle || gamepad1.b) {

                    viper.setStage(
                            ViperArm.Stage.STOWED
                    );

                    presetRequested = true;
                    command = "PRESET STOWED";
                }
            }


            // ============================================================
            // 3. HOLD POSITION
            // ================================================================

            if (!manualControlActive
                    && !presetRequested) {

                viper.holdPosition();

                command = "HOLD";
            }


            // ============================================================
            // 4. READ VIPER STATE
            // ================================================================

            int currentTicks =
                    viper.getCurrentTicks();

            int targetTicks =
                    viper.getTargetTicks();

            int positionError =
                    viper.getPositionError();

            boolean atTarget =
                    viper.isAtTarget();


            // ============================================================
            // 5. TELEMETRY
            // ================================================================

            telemetry.addLine(
                    "========== VIPER SLIDE =========="
            );

            telemetry.addData(
                    "Command",
                    command
            );

            telemetry.addLine();

            telemetry.addData(
                    "Current Position",
                    "%d ticks",
                    currentTicks
            );

            telemetry.addData(
                    "Target Position",
                    "%d ticks",
                    targetTicks
            );

            telemetry.addData(
                    "Position Error",
                    "%d ticks",
                    positionError
            );

            telemetry.addData(
                    "At Target",
                    atTarget ? "YES" : "NO"
            );

            telemetry.addLine();

            telemetry.addData(
                    "Selected Stage",
                    viper.getCurrentStage()
            );

            telemetry.addData(
                    "Stage Target",
                    "%d ticks",
                    viper.getCurrentStage().ticks
            );

            telemetry.addLine();

            telemetry.addData(
                    "Max Allowed",
                    "%d ticks",
                    org.firstinspires.ftc.teamcode
                            .HyperionRobotics.constants
                            .RobotConstants.VIPER_MAX_TICKS
            );

            telemetry.addData(
                    "Viper Power Setting",
                    "%.2f",
                    org.firstinspires.ftc.teamcode
                            .HyperionRobotics.constants
                            .RobotConstants.VIPER_POWER
            );

            telemetry.addLine();

            telemetry.addLine(
                    "---------- GAMEPAD ----------"
            );

            telemetry.addData(
                    "Left Bumper / UP",
                    gamepad1.left_bumper
            );

            telemetry.addData(
                    "Left Trigger / DOWN",
                    "%.2f",
                    gamepad1.left_trigger
            );

            telemetry.addData(
                    "Manual Control",
                    manualControlActive
                            ? "ACTIVE"
                            : "OFF"
            );

            telemetry.addData(
                    "Preset Command",
                    presetRequested
                            ? "YES"
                            : "NO"
            );

            telemetry.update();

            idle();
        }


        // ================================================================
        // STOP VIPER
        // ================================================================

        viper.stop();
    }
}

/*
Commneted out for testing purposes.*************************************************************
package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;
*/
/**
 * TeleOp OpMode: Jarvis Viper
 * 1. Drivetrain: Right stick (FWD/BWD), Left stick (Left/Right turn).
 * 2. Intake: Right Bumper (Toggle ON/OFF).
 * 3. Viper Arm Manual: Left Bumper (UP), Left Trigger (DOWN).
 * 4. Viper Arm Presets: D-pad (Top=1, Right=2, Down=3, Left=4), Circle=0.
 */
/* Commented out for testing purposes.
@TeleOp(name="Jarvis Viper", group="TeleOp")
public class JarvisViper extends LinearOpMode {

    private HyperionRobot robot;
    private boolean intakeActive = false;
    private boolean lastRbState = false;

    @Override
    public void runOpMode() {
        robot = new HyperionRobot(hardwareMap);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. DRIVETRAIN (Arcade Drive) ---
            // Request: Right stick = FWD/BWD, Left stick = Left/Right
            double drive = -gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            robot.drive.arcadeDrive(drive, turn);

            // --- 2. INTAKE (Toggle via Right Bumper) ---
            if (gamepad1.right_bumper && !lastRbState) {
                intakeActive = !intakeActive; // Toggle state
                if (intakeActive) {
                    robot.intake.intake();
                    robot.intake.open();
                } else {
                    robot.intake.stop();
                    robot.intake.close();
                }
            }
            lastRbState = gamepad1.right_bumper;

            // --- 3. VIPER ARM (Manual Control) ---
            if (gamepad1.left_bumper) {
                robot.viper.jog(1.0); // Extend up
            } else if (gamepad1.left_trigger > 0.1) {
                robot.viper.jog(-1.0); // Retract down
            }

            // --- 4. VIPER ARM (Presets via D-pad & Circle) ---
            if (gamepad1.dpad_up) {
                robot.viper.setStage(ViperArm.Stage.LOW); // Stage 1
            } else if (gamepad1.dpad_right) {
                robot.viper.setStage(ViperArm.Stage.MID); // Stage 2
            } else if (gamepad1.dpad_down) {
                robot.viper.setStage(ViperArm.Stage.HIGH); // Stage 3
            } else if (gamepad1.dpad_left) {
                robot.viper.setStage(ViperArm.Stage.MAX); // Stage 4
            } else if (gamepad1.circle || gamepad1.b) {
                robot.viper.setStage(ViperArm.Stage.STOWED); // Stage 0
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("Intake", intakeActive ? "RUNNING" : "OFF");
            telemetry.addData("Viper Ticks", robot.viper.getCurrentTicks());
            telemetry.addData("Viper Stage", robot.viper.getCurrentStage());
            telemetry.update();
        }

        robot.stopAll();
    }
}
*/
