package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake_kit", group = "Linear OpMode")
public class Intake_kit extends LinearOpMode {


    private DcMotor back_left_motor = null;
    private DcMotor back_right_motor = null;


    private DcMotor intake_motor = null;
    private CRServo left_intake_servo = null;
    private CRServo right_intake_servo = null;

    @Override
    public void runOpMode() {


        back_left_motor   = hardwareMap.get(DcMotor.class, " back_left_motor");
        back_right_motor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        intake_motor      = hardwareMap.get(DcMotor.class, "intake_motor");
        left_intake_servo  = hardwareMap.get(CRServo.class, " left_intake_servo");
        right_intake_servo = hardwareMap.get(CRServo.class, "right_intake_servo");


        back_left_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);


        left_intake_servo.setDirection(CRServo.Direction.REVERSE);
        right_intake_servo.setDirection(CRServo.Direction.FORWARD);


        telemetry.addData("Status", "Initialized & Ready!");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {


            double drive = -gamepad1.left_stick_y;  // Push forward to move forward
            double turn  =  gamepad1.right_stick_x; // Left/Right stick to turn sides


            double leftPower  = drive + turn;
            double rightPower = drive - turn;


            leftPower  = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));


            back_left_motor.setPower(leftPower);
            back_right_motor.setPower(rightPower);


            if (gamepad1.right_bumper) {
                left_intake_servo.setPower(1.0);
                right_intake_servo.setPower(1.0);
            } else {
                left_intake_servo.setPower(0.0);
                right_intake_servo.setPower(0.0);
            }


            if (gamepad1.right_trigger > 0.1) {

                intake_motor.setPower(gamepad1.right_trigger);
            } else {
                intake_motor.setPower(0.0);
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("Drive Motors", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.addData("R1 Servos", gamepad1.right_bumper ? "ACTIVE" : "OFF");
            telemetry.addData("R2 Core Motor", "Power: %.2f", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}

