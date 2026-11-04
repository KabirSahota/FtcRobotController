package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Artemis Basic Auton", group = "autonomous")
public class Artemis_BasicAuton extends LinearOpMode {


    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;




    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Based on H/W Config, initialize the S/W variables for the rest of the code to work with.




        // Put initialization blocks here
        waitForStart();
        // Put run blocks here
        while (opModeIsActive()) {
            leftBack.setPower((0.4));
            rightBack.setPower(0.4);
            leftFront.setPower(0.4);
            rightFront.setPower(0.4);
            sleep(2000);
            leftBack.setPower((-0.4));
            rightBack.setPower(-0.4);
            leftFront.setPower(-0.4);
            rightFront.setPower(-0.4);
            sleep(2000);
            leftBack.setPower((0));
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            break;

            // Put loop blocks here
        }
    }




}

