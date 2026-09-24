package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Mecunum_Shooter", group = "TeleOp")
public class Mecunum_Shooter extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor intake;
    private DcMotor hogback;

    private CRServo windmillServo;
    private CRServo outerServoLeft;
    private CRServo outerServoRight;

    private boolean intakeSystemOn = false;
    private boolean lastR1 = false;

    // =========================
    // CRSERVO POWER CONSTANTS
    // =========================

    // Power values for continuous rotation servos (-1.0 to 1.0)
    private static final double WINDMILL_POWER = 1.0;
    private static final double OUTER_LEFT_POWER = 1.0;
    private static final double OUTER_RIGHT_POWER = 1.0;

    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE CONFIG
        // =========================

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hogback = hardwareMap.get(DcMotor.class, "hogback");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        outerServoLeft = hardwareMap.get(CRServo.class, "outerServoLeft");
        outerServoRight = hardwareMap.get(CRServo.class, "outerServoRight");

        // =========================
        // MECANUM MOTOR DIRECTIONS & INITIAL POWERS
        // =========================

        intake.setPower(0.0);
        hogback.setPower(0.0);
        windmillServo.setPower(0.0);
        outerServoLeft.setPower(0.0);
        outerServoRight.setPower(0.0);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // =========================
        // BRAKING
        // =========================

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("HYPERION BIOBUZZ READY");
        telemetry.addLine("MECANUM DRIVE");
        telemetry.addLine("R1 = Toggle Intake System");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            telemetry.addData("Gamepad ID", gamepad1.getGamepadId());
            telemetry.addData("Left Stick", "x=%.2f y=%.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);

            double leftFrontPower = y + x + rotation;
            double rightFrontPower = y - x - rotation;
            double leftBackPower = y - x + rotation;
            double rightBackPower = y + x - rotation;

            // =========================
            // NORMALIZE
            // =========================

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

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // =========================
            // R1 TOGGLE
            // =========================

            boolean currentR1 = gamepad1.right_bumper;

            if (currentR1 && !lastR1) {
                intakeSystemOn = !intakeSystemOn;
            }

            lastR1 = currentR1;

            // =========================
            // INTAKE SYSTEM
            // =========================

            if (intakeSystemOn) {

                // Intake motors
                intake.setPower(1.0);
                hogback.setPower(1.0);

                // Continuous Rotation Servos
                windmillServo.setPower(WINDMILL_POWER);
                outerServoLeft.setPower(OUTER_LEFT_POWER);
                outerServoRight.setPower(OUTER_RIGHT_POWER);

            } else {

                // Stop
                intake.setPower(0.0);
                hogback.setPower(0.0);

                windmillServo.setPower(0.0);
                outerServoLeft.setPower(0.0);
                outerServoRight.setPower(0.0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Intake System", intakeSystemOn ? "ON" : "OFF");
            telemetry.addData("Windmill Power", intakeSystemOn ? WINDMILL_POWER : 0.0);
            telemetry.addData("Outer Servos Power", intakeSystemOn ? OUTER_LEFT_POWER : 0.0);

            telemetry.update();
        }
    }
}