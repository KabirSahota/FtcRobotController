package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Artemis main teleop", group = "TeleOp")
public class Shooter_bot extends LinearOpMode {

    // Windmill CRServo state machine variables (infinite rotate-stop loop)
    private enum WindmillState {
        OFF,
        TURNING,
        PAUSING
    }
    private WindmillState windmillState = WindmillState.OFF;
    private ElapsedTime windmillTimer = new ElapsedTime();

    private static final double WINDMILL_SPEED = -0.30;
    private static final double TURN_DURATION = 1.0;  // seconds rotating
    private static final double PAUSE_DURATION = 0.25; // seconds stopped

    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // =========================
    // INTAKE SYSTEM
    // =========================

    private DcMotor intake;
    private DcMotorEx hogback;

    // Outtake (hogback) velocity control.
    // true  = use encoder-based setVelocity() to hold OUTTAKE_TARGET_RPM
    // false = use the original open-loop setPower()
    private static final boolean VelocityBasedOuttake = true;
    private static final double OUTTAKE_TARGET_RPM = 2950;
    // Set this to the encoder ticks per revolution of the hogback motor. GoBilda 5203 series motor 28 Ticks for one revolution
    private static final double HOGBACK_TICKS_PER_REV = 28;
    // Windmill won't start until the hogback is within this many RPM of the target
    private static final double OUTTAKE_RPM_TOLERANCE = 100;

    private CRServo windmillServo;
    private CRServo outerServoLeft;
    private CRServo outerServoRight;

    // R1 toggle
    private boolean outakeSystemOn = false;
    private boolean intakeSystemOn = false;
    private boolean reverseOuttake = false;
    private boolean lastR1 = false;
    private boolean lastR2 = false;
    private boolean lastL1 = false;

    @Override
    public void runOpMode() {

        // Based on H/W Config, initialize the S/W variables for the rest of the code to work with.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");

        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        outerServoLeft = hardwareMap.get(CRServo.class, "outerServoLeft");
        outerServoRight = hardwareMap.get(CRServo.class, "outerServoRight");

        // Reverse right intake servo so both spin together into the intake
        outerServoRight.setDirection(CRServo.Direction.REVERSE);
        /*
         * When the H/W gets powered up, we want it to be in a known/good state. There's a bunch of
         * methods we can call into in order to do that. Each type of H/W part needs a different set
         * of init routine to be called into.
         */

        // Init for the Mecanum wheels.

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Velocity control needs the encoder to be used by the motor controller.
        if (VelocityBasedOuttake) {
            hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
         * Servo inits are simpler. However, more advanced modes can be be programmed with the
         * servo programmer outside the code.
         */
        windmillServo.setPower(0.0);
        outerServoLeft.setPower(0.0);
        outerServoRight.setPower(0.0);

        //  Print out the controls
        telemetry.addLine("HYPERION BIOBUZZ MECANUM READY");
        telemetry.addLine("Left Stick Y = Forward / Back");
        telemetry.addLine("Left Stick X = Strafe");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("R1 = Toggle Outake System");
        telemetry.addLine("L1 = Toggle Intake System");
        telemetry.update();

        waitForStart();

        int loopCounter = 0;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            // Mecanum wheel calculations
            double leftFrontPower  = y + x + rotation;
            double rightFrontPower = y - x - rotation;
            double leftBackPower   = y - x + rotation;
            double rightBackPower  = y + x - rotation;

            /*
             * The values on the gamepad cannnot directly be input to the motor. They need
             * to be relative values to the maximum value of power that can be applied to the
             * motor which is 1.0.
             */
            double maxPower = Math.max(
                    1.0,
                    Math.max(
                            Math.abs(leftFrontPower),
                            Math.max(
                                    Math.abs(rightFrontPower),
                                    Math.max(
                                            Math.abs(leftBackPower),
                                            Math.abs(rightBackPower)
                                    )
                            )
                    )
            );

            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;

            // SET DRIVE POWER
            leftFront.setPower(leftFrontPower * 0.8);
            rightFront.setPower(rightFrontPower * 0.8);
            leftBack.setPower(leftBackPower * 0.8);
            rightBack.setPower(rightBackPower * 0.8);

            /*
             * Process the input from the bumper now.
             */
            boolean currentR1 = gamepad1.right_bumper;

            // Only toggle once per button press
            if (currentR1 && !lastR1) {
                outakeSystemOn = !outakeSystemOn;
            }
            lastR1 = currentR1;

            // Measured hogback speed (ticks/sec -> RPM)
            double hogbackRpm = hogback.getVelocity() * 60.0 / HOGBACK_TICKS_PER_REV;
            boolean hogbackAtTargetRpm =
                    Math.abs(hogbackRpm - OUTTAKE_TARGET_RPM) <= OUTTAKE_RPM_TOLERANCE;

            // Outtake SYSTEM
            if (outakeSystemOn) {
                // Emergency outtake & windmill reversal button if it jams
                boolean currentR2 = gamepad1.a;
                if (currentR2 && !lastR2) {
                    reverseOuttake = !reverseOuttake;
                }
                lastR2 = currentR2;

                if (reverseOuttake)  {
                    windmillServo.setPower(-1.0);
                    hogback.setPower(-0.5);
                } else {
                    windmillServo.setPower(1.0);
                    // Hogback wheel
                    if (VelocityBasedOuttake) {
                        // Convert RPM to encoder ticks per second and let the encoder hold it
                        hogback.setVelocity(OUTTAKE_TARGET_RPM * HOGBACK_TICKS_PER_REV / 60.0);
                    } else {
                        hogback.setPower(0.8);
                    }
                }
            } else {
                // Stop hogback
                hogback.setPower(0.0);

                // Stop windmill CRServo and reset state
                windmillServo.setPower(0.0);
                windmillState = WindmillState.OFF;

                // Stop continuous rotation intake servos
                outerServoLeft.setPower(0.0);
                outerServoRight.setPower(0.0);
            }

            // Run the intake system based on the left bumper
            boolean currentL1 = gamepad1.left_bumper;

            if (currentL1 && !lastL1) {
                intakeSystemOn = !intakeSystemOn;
            }

            lastL1 = currentL1;
            if (intakeSystemOn) {
                intake.setPower(1.0);
                // Continuous rotation intake servos
                outerServoLeft.setPower(1.0);
                outerServoRight.setPower(1.0);
            } else {
                intake.setPower(0.0);
                outerServoLeft.setPower(0.0);
                outerServoRight.setPower(0.0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Loop Count", loopCounter++);
            telemetry.addData("Stick X", x);
            telemetry.addData("Stick Y", y);
            telemetry.addLine("reversing the windmill is x button");
            telemetry.addData("Rotation", rotation);

            telemetry.addData("Front Left", leftFrontPower);
            telemetry.addData("Front Right", rightFrontPower);
            telemetry.addData("Back Left", leftBackPower);
            telemetry.addData("Back Right", rightBackPower);

            telemetry.addData("Intake System", intakeSystemOn ? "ON" : "OFF");
            telemetry.addData("Outtake System", outakeSystemOn ? "ON" : "OFF");
            telemetry.addData("Hogback RPM", hogbackRpm);
            telemetry.addData("Hogback Target RPM", OUTTAKE_TARGET_RPM);

            telemetry.update();
        }
    }
}