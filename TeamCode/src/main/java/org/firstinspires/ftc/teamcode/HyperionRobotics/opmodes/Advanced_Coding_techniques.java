
package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Advanced_Coding_techniques")
public class Advanced_Coding_techniques extends LinearOpMode {

    DcMotor leftFront, rightFront;
    DcMotor leftBack, rightBack;

    DcMotor intake, hogback;

    static final double COUNTS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_CM = 9.6;

    static final double COUNTS_PER_CM =
            COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_CM);

    // Adjust this after testing your robot
    static final double TURN_TICKS_90 = 500;

    @Override
    public void runOpMode() {

        // Initialize drivetrain
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize intake and outtake
        intake = hardwareMap.get(DcMotor.class, "intake");
        hogback = hardwareMap.get(DcMotor.class, "hogback");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        resetEncoders();

        waitForStart();

        if (isStopRequested()) return;


        driveForward(15, 0.5);


        sleep(1000);


        turnRight90();


        intake.setPower(1.0);

        // 5. Drive forward 15 cm
        driveForward(15, 0.4);

        // 6. Turn off intake and activate outtake
        intake.setPower(0);
        hogback.setPower(1.0);

        sleep(1000);

        // 7. Stop everything
        stopAll();
    }

    // Reset drivetrain encoders
    private void resetEncoders() {

        DcMotor[] motors = {
                leftFront, rightFront, leftBack, rightBack
        };

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Drive forward a measured distance in centimeters
    private void driveForward(double cm, double speed) {

        int ticks = (int)(cm * COUNTS_PER_CM);

        setTargetPositions(ticks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(speed, speed);

        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {

            telemetry.addData("Left Front", leftFront.getCurrentPosition());
            telemetry.addData("Right Front", rightFront.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Turn right approximately 90 degrees
    private void turnRight90() {

        setTargetPositions(
                (int) -TURN_TICKS_90,
                (int) -TURN_TICKS_90,
                (int) -TURN_TICKS_90,
                (int) -TURN_TICKS_90
        );

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(0.35, 0.35);

        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftBack.isBusy() || rightBack.isBusy())) {
            telemetry.addData("Turning", "Right 90");
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set same target for all four motors
    private void setTargetPositions(int ticks) {
        setTargetPositions(ticks, ticks, ticks, ticks);
    }

    // Set individual motor targets
    private void setTargetPositions(
            int lf, int rf, int lb, int rb) {

        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + lf);

        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + rf);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + lb);

        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + rb);
    }

    // Set left and right drive power
    private void setDrivePower(double left, double right) {

        leftFront.setPower(left);
        leftBack.setPower(left);

        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    // Set drivetrain mode
    private void setDriveMode(DcMotor.RunMode mode) {

        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    // Stop drivetrain
    private void stopDrive() {
        setDrivePower(0, 0);
    }

    // Stop all motors
    private void stopAll() {

        stopDrive();
        intake.setPower(0);
        hogback.setPower(0);
    }
}

