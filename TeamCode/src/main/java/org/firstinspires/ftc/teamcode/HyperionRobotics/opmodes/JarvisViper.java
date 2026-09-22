package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobotTeleOp;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

@TeleOp(name = "JarvisViper", group = "TeleOp")
public class JarvisViper extends LinearOpMode {

    private HyperionRobotTeleOp robot;

    private boolean intakeActive = false;
    private boolean lastRbState = false;

    @Override
    public void runOpMode() {

        robot = new HyperionRobotTeleOp(hardwareMap);

        telemetry.addLine("JARVIS TELEOP");
        telemetry.addLine("Initialized. Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopAll();
            return;
        }

        while (opModeIsActive()) {

            // ============================================================
            // 1. DRIVETRAIN
            // ============================================================

            // LEFT STICK Y = FORWARD / BACKWARD
            double drive = -gamepad1.left_stick_y;

            // RIGHT STICK X = TURN LEFT / RIGHT
            double turn = gamepad1.right_stick_x;

            robot.drive.arcadeDrive(drive, turn);


            // ============================================================
            // 2. INTAKE
            // ============================================================

            // Right bumper toggles intake
            if (gamepad1.right_bumper && !lastRbState) {

                intakeActive = !intakeActive;

                if (intakeActive) {
                    robot.intake.intake();
                    robot.intake.open();
                } else {
                    robot.intake.stop();
                    robot.intake.close();
                }
            }

            lastRbState = gamepad1.right_bumper;


            // ============================================================
            // 3. VIPER MANUAL CONTROL
            // ============================================================

            // Left bumper = UP
            if (gamepad1.left_bumper) {

                robot.viper.jog(1.0);

                // Left trigger = DOWN
            } else if (gamepad1.left_trigger > 0.1) {

                robot.viper.jog(-1.0);
            }


            // ============================================================
            // 4. VIPER PRESETS
            // ============================================================

            if (gamepad1.dpad_up) {

                robot.viper.setStage(ViperArm.Stage.LOW);

            } else if (gamepad1.dpad_right) {

                robot.viper.setStage(ViperArm.Stage.MID);

            } else if (gamepad1.dpad_down) {

                robot.viper.setStage(ViperArm.Stage.HIGH);

            } else if (gamepad1.dpad_left) {

                robot.viper.setStage(ViperArm.Stage.MAX);

            } else if (gamepad1.circle || gamepad1.b) {

                robot.viper.setStage(ViperArm.Stage.STOWED);
            }


            // ============================================================
            // 5. TELEMETRY
            // ============================================================

            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick X");

            telemetry.addData(
                    "Intake",
                    intakeActive ? "RUNNING" : "OFF"
            );

            telemetry.addData(
                    "Viper Ticks",
                    robot.viper.getCurrentTicks()
            );

            telemetry.addData(
                    "Viper Stage",
                    robot.viper.getCurrentStage()
            );

            telemetry.update();
        }

        // Stop everything
        robot.stopAll();
    }
}