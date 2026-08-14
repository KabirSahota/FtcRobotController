
package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

/**
 * Viper Motor + Encoder Diagnostic
 *
 * PURPOSE:
 *   Test the motor and encoder separately from ViperArm.
 *
 * IMPORTANT:
 *   This OpMode uses RUN_WITHOUT_ENCODER.
 *
 *   The encoder is ONLY READ for telemetry.
 *   It is NOT being used to control the motor.
 *
 * CONTROLS:
 *
 *   LEFT BUMPER  = Motor +0.15
 *   LEFT TRIGGER = Motor -0.15
 *
 *   Release controls = MOTOR STOP
 *
 *   A = Reset encoder count to zero
 *
 * SAFETY:
 *   Start with the slide away from both hard stops.
 *   Keep a hand near STOP.
 *   Use only short movements at first.
 */
@TeleOp(
        name = "Viper Motor Encoder Debug",
        group = "Diagnostics"
)
public class ViperMotorEncoderDebug extends LinearOpMode {

    private DcMotorEx viperMotor;

    /*
     * Keep this intentionally low for the first tests.
     */
    private static final double TEST_POWER = 0.50;

    private int previousTicks = 0;
    private long previousTimeMs = 0;

    private boolean lastA = false;

    @Override
    public void runOpMode() {

        // ================================================================
        // GET ONLY THE VIPER MOTOR
        // ================================================================

        viperMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        RobotConstants.VIPER_MOTOR
                );

        viperMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        // ================================================================
        // MOTOR CONFIGURATION
        // ================================================================

        /*
         * Brake when power goes to zero.
         */
        viperMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        /*
         * Reset encoder once at initialization.
         */
        viperMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        /*
         * CRITICAL:
         *
         * Do NOT use RUN_TO_POSITION here.
         *
         * RUN_WITHOUT_ENCODER means motor power is controlled directly.
         *
         * We can still READ getCurrentPosition().
         */
        viperMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        viperMotor.setPower(0.0);

        previousTicks =
                viperMotor.getCurrentPosition();

        previousTimeMs =
                System.currentTimeMillis();


        // ================================================================
        // INITIAL TELEMETRY
        // ================================================================

        telemetry.addLine(
                "===================================="
        );

        telemetry.addLine(
                "   VIPER MOTOR + ENCODER DEBUG"
        );

        telemetry.addLine(
                "===================================="
        );

        telemetry.addLine();

        telemetry.addLine(
                "LEFT BUMPER  = +0.50 MOTOR"
        );

        telemetry.addLine(
                "LEFT TRIGGER = -0.50 MOTOR"
        );

        telemetry.addLine(
                "A = RESET ENCODER"
        );

        telemetry.addLine();

        telemetry.addLine(
                "Release controls = MOTOR STOP"
        );

        telemetry.addLine();

        telemetry.addLine(
                "Start with SHORT movements."
        );

        telemetry.addLine(
                "Keep robot STOP accessible."
        );

        telemetry.update();


        waitForStart();


        if (isStopRequested()) {

            viperMotor.setPower(0.0);
            return;
        }


        // ================================================================
        // MAIN LOOP
        // ================================================================

        while (opModeIsActive()) {

            double commandPower = 0.0;

            String command =
                    "STOP";


            // ============================================================
            // MOTOR CONTROL
            // ============================================================

            if (gamepad1.left_bumper) {

                commandPower =
                        TEST_POWER;

                command =
                        "FORWARD / +POWER";

            } else if (gamepad1.left_trigger > 0.1) {

                commandPower =
                        -TEST_POWER;

                command =
                        "REVERSE / -POWER";
            }


            /*
             * Direct motor power.
             */
            viperMotor.setPower(
                    commandPower
            );


            // ============================================================
            // RESET ENCODER WITH A BUTTON
            // ============================================================

            boolean currentA =
                    gamepad1.a;

            if (currentA && !lastA) {

                /*
                 * Stop before changing motor mode.
                 */
                viperMotor.setPower(
                        0.0
                );

                viperMotor.setMode(
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );

                viperMotor.setMode(
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );

                previousTicks =
                        0;

                previousTimeMs =
                        System.currentTimeMillis();
            }

            lastA =
                    currentA;


            // ============================================================
            // READ ENCODER
            // ============================================================

            int currentTicks =
                    viperMotor.getCurrentPosition();

            long currentTimeMs =
                    System.currentTimeMillis();

            int tickChange =
                    currentTicks
                            - previousTicks;

            long timeChangeMs =
                    currentTimeMs
                            - previousTimeMs;


            /*
             * Approximate encoder speed.
             *
             * This is only diagnostic.
             */
            double ticksPerSecond =
                    0.0;

            if (timeChangeMs > 0) {

                ticksPerSecond =
                        tickChange
                                * 1000.0
                                / timeChangeMs;
            }


            // ============================================================
            // SIMPLE ENCODER HEALTH INTERPRETATION
            // ============================================================

            String encoderStatus;

            if (Math.abs(commandPower) < 0.01) {

                encoderStatus =
                        "MOTOR STOPPED";

            } else if (Math.abs(tickChange) == 0) {

                encoderStatus =
                        "NO ENCODER CHANGE";

            } else {

                encoderStatus =
                        "ENCODER CHANGING";
            }


            // ============================================================
            // TELEMETRY
            // ============================================================

            telemetry.addLine(
                    "========== MOTOR =========="
            );

            telemetry.addData(
                    "Command",
                    command
            );

            telemetry.addData(
                    "Power Command",
                    "%.2f",
                    commandPower
            );

            telemetry.addData(
                    "Run Mode",
                    viperMotor.getMode()
            );

            telemetry.addLine();


            telemetry.addLine(
                    "========== ENCODER =========="
            );

            telemetry.addData(
                    "Current Ticks",
                    currentTicks
            );

            telemetry.addData(
                    "Previous Ticks",
                    previousTicks
            );

            telemetry.addData(
                    "Tick Change",
                    tickChange
            );

            telemetry.addData(
                    "Approx Ticks/sec",
                    "%.1f",
                    ticksPerSecond
            );

            telemetry.addData(
                    "Encoder Status",
                    encoderStatus
            );

            telemetry.addLine();


            telemetry.addLine(
                    "========== GAMEPAD =========="
            );

            telemetry.addData(
                    "Left Bumper",
                    gamepad1.left_bumper
            );

            telemetry.addData(
                    "Left Trigger",
                    "%.2f",
                    gamepad1.left_trigger
            );

            telemetry.addData(
                    "A / Reset",
                    gamepad1.a
            );

            telemetry.update();


            // ============================================================
            // SAVE VALUES FOR NEXT LOOP
            // ============================================================

            previousTicks =
                    currentTicks;

            previousTimeMs =
                    currentTimeMs;


            idle();
        }


        // ================================================================
        // ALWAYS STOP MOTOR WHEN OPMODE ENDS
        // ================================================================

        viperMotor.setPower(
                0.0
        );
    }
}
