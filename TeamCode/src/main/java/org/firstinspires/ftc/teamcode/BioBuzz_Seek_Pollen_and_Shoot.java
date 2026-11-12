package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Autonomous(name="BioBuzz_Seek_Pollen_and_Shoot", group="Robot")
public class BioBuzz_Seek_Pollen_and_Shoot extends LinearOpMode {

    // Hardware definitions using your exact configurations
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake, hogback;

    // Official FTC Limelight Driver Object
    private Limelight3A limelight;

    // States for the sequence
    private enum State {
        SEEK_POLLEN,
        INTAKE_POLLEN,
        SHOOT_POLLEN,
        DONE
    }

    private State currentState = State.SHOOT_POLLEN;

    // Proportional constants for tracking
    private final double kP_Turn = 0.04;
    private final double kP_Drive = 0.05;
    private final double TARGET_AREA = 12.0; // Adjust this % based on your intake distance

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize Mechanisms
        intake = hardwareMap.get(DcMotor.class, "intake");
        hogback = hardwareMap.get(DcMotor.class, "hogback");

        // Initialize the Limelight hardware object
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Reverse left side motors so positive power moves the robot forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        limelight.setPollRateHz(100);

        limelight.start();

        // Match this index with your custom Pollen pipeline (usually pipeline 0 or 1)
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized. Errors fixed! Ready for BioBuzz. also i like food");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && currentState != State.SHOOT_POLLEN) {

            // Fetch the freshest tracking result from the Limelight object
            LLResult result = limelight.getLatestResult();

            boolean targetVisible = (result != null && result.isValid());
            double horizontalOffset = targetVisible ? result.getTx() : 0.0;
            double currentArea = targetVisible ? result.getTa() : 0.0;

            switch (currentState) {
                case SEEK_POLLEN:
                    if (targetVisible) {
                        // Calculate steering and forward drive values
                        double turnPower = horizontalOffset * kP_Turn;
                        double drivePower = (TARGET_AREA - currentArea) * kP_Drive;

                        // Clip powers to maintain control and safety
                        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
                        drivePower = Math.max(-0.5, Math.min(0.5, drivePower));

                        // Apply tank-style power distributing to all 4 motors
                        leftFront.setPower(drivePower + turnPower);
                        leftBack.setPower(drivePower + turnPower);
                        rightFront.setPower(drivePower - turnPower);
                        rightBack.setPower(drivePower - turnPower);

                        // Stop when the pollen ball fills the field of view enough to intake
                        if (currentArea >= TARGET_AREA) {
                            stopDrivetrain();
                            currentState = State.INTAKE_POLLEN;
                        }
                    } else {
                        // Rotate slowly in place to look for pollen if lost
                        leftFront.setPower(0.2);
                        leftBack.setPower(0.2);
                        rightFront.setPower(-0.2);
                        rightBack.setPower(-0.2);
                    }
                    break;

                case INTAKE_POLLEN:
                    telemetry.addData("State", "Running Intake...");
                    telemetry.update();

                    // Activate intake to bring the ball into the robot
                    intake.setPower(1.0);
                    sleep(2000);
                    intake.setPower(0.0);

                    currentState = State.SHOOT_POLLEN;
                    break;

                case SHOOT_POLLEN:
                    telemetry.addData("State", "Spinning up Hogback & Firing...");
                    telemetry.update();

                    // Spin up the outtake flywheel (hogback)
                    hogback.setPower(1.0);
                    sleep(1500); // Allow time to reach full launching velocity

                    // Use the intake to feed the collected pollen upward into the hogback
                    intake.setPower(1.0);
                    sleep(2000);

                    // Turn off all motors
                    hogback.setPower(0.0);
                    intake.setPower(0.0);

                    currentState = State.DONE;
                    break;

                case DONE:
                    break;
            }
        }

        // Always stop camera streaming when OpMode finishes
        limelight.stop();
    }

    // Helper method to completely cut power to the wheels
    private void stopDrivetrain() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

}