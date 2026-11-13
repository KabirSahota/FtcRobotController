package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Artemis Timed Distance - Blue", group = "autonomous")
public class Artemis_BasicAuton extends LinearOpMode {


    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFront;
    private DcMotor hogback;
    private DcMotor windmillMotor;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;




    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        hogback = hardwareMap.get(DcMotor.class, "hogback");
       windmillMotor = hardwareMap.get(DcMotor.class, "windmillMotor");
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
    if (opModeIsActive()) {
        drive(0.4, 1625);   // Drive forward
        turn(0.4, 700);// Turn and stuff
     outake(15000);
        turn(-0.4,750);
        drive(-0.4, 1400);  // Drive backward

    }
}

// =========================
// Fuunctiionsss
// =========================

private void drive(double power, int milliseconds) {
    leftFront.setPower(power);
    leftBack.setPower(power);
    rightFront.setPower(power);
    rightBack.setPower(power);
    sleep(milliseconds);
    stopdrive();
}
    private void outake( int milliseconds) {
        hogback.setPower (0.45);
        sleep(2000);
        windmillMotor.setPower (-0.4);
        sleep(milliseconds);
        stopoutake();
    }

private void turn(double power, int milliseconds) {
    leftFront.setPower(power);
    leftBack.setPower(power);
    rightFront.setPower(-power);
    rightBack.setPower(-power);
    sleep(milliseconds);
    stopdrive();
}

private void stopdrive() {
    leftFront.setPower(0);
    leftBack.setPower(0);
    rightFront.setPower(0);
    rightBack.setPower(0);
}
    private void stopoutake() {
         windmillMotor.setPower(0);
        hogback.setPower(0);
    }

}

