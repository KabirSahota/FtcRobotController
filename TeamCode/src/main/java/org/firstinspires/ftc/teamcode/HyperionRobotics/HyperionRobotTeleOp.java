package org.firstinspires.ftc.teamcode.HyperionRobotics;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HyperionRobotics.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.HyperionRobotics.intake.Intake;
import org.firstinspires.ftc.teamcode.HyperionRobotics.viper.ViperArm;

/**
 * Aggregates all HyperionRobotics subsystems for OpModes.
 */
public class HyperionRobotTeleOp {
    public final DriveTrain drive;
    public final Intake intake;
    public final ViperArm viper;

    public HyperionRobotTeleOp(HardwareMap hardwareMap) {
        drive = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        viper = new ViperArm(hardwareMap);
    }


    public void stopAll() {
        drive.stop();
        intake.stop();
        viper.stop();
    }
}
