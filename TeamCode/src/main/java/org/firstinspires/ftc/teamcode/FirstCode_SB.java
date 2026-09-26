package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// @TeleOp(name = "FirstCode_SB")
public class FirstCode_SB extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"front_left_motor");
        DcMotor backLeft = hardwareMap.get(DcMotor.class,"back_left_motor");
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"front_right_motor");
        DcMotor backRight = hardwareMap.get(DcMotor.class,"back_right_motor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double flPower = drive + strafe + turn;
            double blPower = drive - strafe + turn;
            double frPower = drive - strafe - turn;
            double brPower = drive + strafe - turn;

            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            if (max > 1.0) {
                flPower /= max;
                blPower /= max;
                frPower /= max;
                brPower /= max;
            }

            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

        }









        }
}
