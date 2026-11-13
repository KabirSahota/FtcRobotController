
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Diagnostic")
public class PinpointDiag extends LinearOpMode {

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver odo =
                hardwareMap.get(
                        GoBildaPinpointDriver.class, "odo");

        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD);

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.setOffsets(-196.0, 107.0, DistanceUnit.MM);

        telemetry.addLine("Ready for Pinpoint diagnostic");
        telemetry.update();

        waitForStart();

        odo.resetPosAndIMU();

        while (opModeIsActive()) {

            odo.update();

            telemetry.addData("Status",
                    odo.getDeviceStatus());

            telemetry.addData("Encoder X",
                    odo.getEncoderX());

            telemetry.addData("Encoder Y",
                    odo.getEncoderY());

            telemetry.addData("Position X (mm)",
                    odo.getPosX(DistanceUnit.MM));

            telemetry.addData("Position Y (mm)",
                    odo.getPosY(DistanceUnit.MM));

            telemetry.addData("Heading (degrees)",
                    odo.getHeading(AngleUnit.DEGREES));

            telemetry.update();

            sleep(50);
        }
    }
}
