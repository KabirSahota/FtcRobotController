package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * Sample autonomous: drive out, correct pose with Limelight, score sequence stub.
 * Replace waypoints / tag IDs for your alliance starting position.
 */
@Autonomous(name = "Hyperion Auto Drive", group = "Hyperion")
public class HyperionAutoDrive extends LinearOpMode {
    private HyperionRobot robot;
    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;


    @Override
    public void runOpMode() {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Based on H/W Config, initialize the S/W variables for the rest of the code to work with.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        leftBack.setPower((0.4));
        rightBack.setPower(0.4);
        leftFront.setPower(0.4);
        rightFront.setPower(0.4);
        sleep(700);
        leftBack.setPower((-0.4));
        rightBack.setPower(-0.4);
        leftFront.setPower(-0.4);
        rightFront.setPower(-0.4);
        sleep(700);
        leftBack.setPower((0));
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        // Put initialization blocks here
        waitForStart();
        // Put run blocks here
        while (opModeIsActive()) {
            // Put loop blocks here
}}}
