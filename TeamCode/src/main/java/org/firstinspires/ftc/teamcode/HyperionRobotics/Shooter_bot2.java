package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Artemis main teleop p2", group = "TeleOp")
public class Shooter_bot2 extends LinearOpMode {
    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
//intake & outake(eventually)
    private DcMotor intake;
    private DcMotor hogback;
    private boolean intakeSystemOn = false;
    private boolean lastL1 = false;

    @Override
    public void runOpMode() {

        // Based on H/W Config, initialize the S/W variables for the rest of the code to work with.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

         /* When the H/W gets powered up, we want it to be in a known/good state. There's a bunch of
         * methods we can call into in order to do that. Each type of H/W part needs a different set
         * of init routine to be called into.
         */// Init for the Mecanum wheels.

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  Print out the controls
        telemetry.addLine("HYPERION BIOBUZZ MECANUM READY");
        telemetry.addLine("Left Stick Y = Forward / Back");
        telemetry.addLine("Left Stick X = Strafe");
        telemetry.addLine("Right Stick X = Turn");
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
            leftBack.setPower(leftBackPower * 0.8 );
            rightBack.setPower(rightBackPower * 0.8);

            /*
             * Process the input from the bumper now.
             */
            boolean currentL1 = gamepad1.left_bumper;

            if (currentL1 && !lastL1) {
                intakeSystemOn = !intakeSystemOn;
            }

            lastL1 = currentL1;
            if (intakeSystemOn) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Loop Count", loopCounter++);
            telemetry.addData("Stick X", x);
            telemetry.addData("Stick Y", y);
            telemetry.addData("Rotation", rotation);

            telemetry.addData("Front Left", leftFrontPower);
            telemetry.addData("Front Right", rightFrontPower);
            telemetry.addData("Back Left", leftBackPower);
            telemetry.addData("Back Right", rightBackPower);



            telemetry.update();
        }
    }
}