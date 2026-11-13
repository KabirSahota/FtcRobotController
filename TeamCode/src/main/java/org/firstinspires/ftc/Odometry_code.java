package org.firstinspires.ftc;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;

/**
 * FTC autonomous for a four-wheel mecanum/Strafer robot with goBILDA Pinpoint.
 *
 * Route: forward 762 mm -> turn right 30 degrees -> shoot -> return to initial
 * heading -> backward 662 mm -> strafe right 304 mm.
 *
 * Key changes:
 * - Smooth PD + friction-compensated turns, rather than a hard 0.18-power
 *   minimum that repeatedly jerks across the target heading.
 * - No extra stop/settle turns at the end of each drive/strafe command.
 * - Elapsed time, total remaining time, and phase time on EVERY motion loop.
 * - Persistent phase-completion timestamps in Driver Station telemetry log.
 * - Shooting reserves 13 seconds for the return, but this is NOT a hard
 *   13-second return timer. FTC stops the OpMode at the end of autonomous.
 * - Backward travel locks to the original field heading and uses a small,
 *   capped correction toward the original field line (not aggressive strafing).
 *
 * Requires the existing team's IntakeControl class in the same package.
 */
@Autonomous(name = "Odometry_code")
public class Odometry_code extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack, hogback;
    private CRServo windmill;
    private GoBildaPinpointDriver odo;
    private final IntakeControl intake = new IntakeControl();

    // Confirm these using the actual pod geometry.
    // X: forward pod's sideways offset (+ left); Y: sideways pod's
    // fore/aft offset (+ forward). Both are relative to tracking center.
    private static final double X_POD_OFFSET_MM = 0.0;
    private static final double Y_POD_OFFSET_MM = -76.0;

    private static final double OUTBOUND_MM = 762.0;
    private static final double SHOOT_RIGHT_DEGREES = 30.0;
    private static final double RETURN_BACKWARD_MM = 625.0;
    private static final double RETURN_STRAFE_RIGHT_MM = 900.0;

    private static final double HOGBACK_POWER = 0.50;
    private static final double WINDMILL_POWER = 0.50;
    private static final double AUTO_DURATION_SEC = 30.0;
    // Adjust after measuring return duration; this only shortens shooting.
    private static final double RETURN_RESERVE_SEC = 13.0;
    private static final double MAX_SHOOT_SEC = 8.0;

    // Keep the previously reasonably straight forward/reverse settings.
    private static final double TRANSLATION_KP = 0.0035;   // power per mm
    private static final double MOVE_MAX_POWER = 0.35;
    private static final double MOVE_MIN_POWER = 0.22;
    private static final double MOVE_TOLERANCE_MM = 18.0;
    private static final double MOVE_HEADING_KP = 0.020;
    private static final double MOVE_MAX_TURN = 0.14;

    // Conservative forward lateral correction; leave OFF for this test.
    // Backward lateral correction is always OFF in this version.
    private static final boolean ENABLE_FORWARD_LATERAL_CORRECTION = false;
    private static final double FORWARD_LATERAL_KP = 0.0010;
    private static final double FORWARD_LATERAL_MAX = 0.04;

    // Gentle return-path correction, measured against the ORIGINAL field line.
    // Positive lateral displacement means left of that original line.
    private static final double BACKWARD_LATERAL_KP = 0.0010;
    private static final double BACKWARD_LATERAL_MAX = 0.035;

    // Smooth turning: angle P + static-friction feedforward - rate damping.
    // No hard minimum 0.18 power near the target (the old source of jitter).
    // These are initial tuning values, not yet calibrated on your robot.
    private static final double TURN_KP = 0.0135;         // power/degree
    private static final double TURN_KD = 0.0015;         // power/(degree/sec)
    private static final double TURN_STATIC_FF = 0.18;    // friction compensation
    private static final double TURN_FF_WIDTH_DEG = 3.0;
    private static final double TURN_MAX_POWER = 0.40;
    private static final double TURN_SLEW_POWER_PER_SEC = 2.8;
    private static final double TURN_TOLERANCE_DEG = 3.0;
    private static final double TURN_SETTLE_RATE_DEG_PER_SEC = 15.0;
    private static final double TURN_SETTLE_SEC = 0.10;
    private static final double TURN_RATE_FILTER = 0.35;

    // Only a no-progress safety check; NO maximum duration per move.
    private static final double STALL_SEC = 1.8;
    private static final double MIN_PROGRESS_MM = 2.0;
    private static final long CONTROL_PERIOD_MS = 20L;

    private ElapsedTime autoTimer;
    private String currentPhase = "Not started";
    private String lastFailure = "None";
    private String lastCompletedStep = "None";
    private double returnStartSec = Double.NaN;
    // Captured ONCE at autonomous start; never overwritten by later turns.
    private double homeHeadingDeg;
    private double homeXmm;
    private double homeYmm;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        hogback = hardwareMap.get(DcMotor.class, "hogback");
        windmill = hardwareMap.get(CRServo.class, "windmillServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setPower(0);
        windmill.setPower(0);
        intake.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.resetPosAndIMU();
        odo.update();
        telemetry.addData("Pinpoint", odo.getDeviceStatus());
        telemetry.addLine("Initialized; verify READY and use a charged battery");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopEverything();
            return;
        }

        autoTimer = new ElapsedTime();
        autoTimer.reset();
        String result = "Interrupted";

        try {
            odo.update();
            if (!pinpointReady()) {
                result = "Pinpoint not READY";
                return;
            }
            homeHeadingDeg = odo.getHeading(AngleUnit.DEGREES);
            homeXmm = odo.getPosX(DistanceUnit.MM);
            homeYmm = odo.getPosY(DistanceUnit.MM);
            final double homeHeading = homeHeadingDeg;

            telemetry.log().add(String.format(Locale.US,
                    "HOME heading=%.2f deg, X=%.1f mm, Y=%.1f mm",
                    homeHeadingDeg, homeXmm, homeYmm));

            if (!moveAxis("1 Forward 762 mm", OUTBOUND_MM, false)) {
                result = "Forward failed: " + lastFailure;
                return;
            }
            if (!turnToHeading("2 Right turn to shoot", homeHeading - SHOOT_RIGHT_DEGREES)) {
                result = "Shoot turn failed: " + lastFailure;
                return;
            }

            shootWithRemainingTime();
            if (!opModeIsActive()) {
                result = "Stopped after shooting";
                return;
            }

            returnStartSec = autoTimer.seconds();
            telemetry.log().add(String.format(Locale.US,
                    "RETURN START t=%.2fs, auto left=%.2fs",
                    returnStartSec, autoRemaining()));

            if (!turnToHeading("4 Restore home heading", homeHeading)) {
                result = "Return turn failed: " + lastFailure;
                return;
            }
            if (!moveAxis("5 Backward 625 mm", -RETURN_BACKWARD_MM, false)) {
                result = "Backward failed: " + lastFailure;
                return;
            }
            if (!moveAxis("6 Strafe right 900 mm", -RETURN_STRAFE_RIGHT_MM, true)) {
                result = "Strafe failed: " + lastFailure;
                return;
            }
            result = "Autonomous finished";
        } finally {
            stopEverything();
            odo.update();
            telemetry.log().add("RESULT: " + result);
            telemetry.addData("Result", result);
            telemetry.addData("Last completed", lastCompletedStep);
            telemetry.addData("Last failure", lastFailure);
            telemetry.addData("Final X (mm)", "%.1f", odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Final Y (mm)", "%.1f", odo.getPosY(DistanceUnit.MM));
            telemetry.addData("Final heading deg", "%.1f", odo.getHeading(AngleUnit.DEGREES));
            showTime("Finished", autoTimer.seconds());
            telemetry.update();
        }
    }

    /** Single axis command (+forward / -backward or +left / -right). */
    private boolean moveAxis(String phase, double amountMm, boolean strafe) {
        currentPhase = phase;
        lastFailure = "None";
        double phaseStart = autoTimer.seconds();
        if (!opModeIsActive()) return failStep(phase, "OpMode stopped", phaseStart);
        odo.update();
        if (!pinpointReady()) return failStep(phase, "Pinpoint not READY", phaseStart);

        Pose2D start = odo.getPosition();
        double startX = start.getX(DistanceUnit.MM);
        double startY = start.getY(DistanceUnit.MM);
        // All axis commands use the same field direction captured at START.
        // A return turn may finish 1-3 degrees off: don't make that error the
        // new "straight" direction for reversing or strafing.
        double targetHeading = homeHeadingDeg;
        double theta = Math.toRadians(targetHeading);
        double direction = Math.signum(amountMm);
        double distance = Math.abs(amountMm);
        if (distance <= MOVE_TOLERANCE_MM) {
            logStep(phase, "Skipped (short move)", phaseStart);
            return true;
        }

        // Signed field travel unit vector for desired axis.
        double ux = (strafe ? -Math.sin(theta) : Math.cos(theta)) * direction;
        double uy = (strafe ?  Math.cos(theta) : Math.sin(theta)) * direction;
        double bestRemaining = distance;
        ElapsedTime progressTimer = new ElapsedTime();
        boolean success = false;
        String reason = "OpMode stopped";

        try {
            while (opModeIsActive()) {
                odo.update();
                if (!pinpointReady()) { reason = "Pinpoint not READY"; break; }

                Pose2D pos = odo.getPosition();
                double dx = pos.getX(DistanceUnit.MM) - startX;
                double dy = pos.getY(DistanceUnit.MM) - startY;
                double heading = pos.getHeading(AngleUnit.DEGREES);
                double progress = dx * ux + dy * uy;
                double remaining = distance - progress;
                double crossTrack = -dx * uy + dy * ux;
                double headingError = normalizeAngle(targetHeading - heading);
                // Lateral displacement relative to the original start line;
                // positive means left of that line, negative means right.
                double lateralFromHome =
                        -(pos.getX(DistanceUnit.MM) - homeXmm) * Math.sin(theta)
                                + (pos.getY(DistanceUnit.MM) - homeYmm) * Math.cos(theta);

                // Stop on requested axis, without additional stop-start heading
                // correction at the end. Heading is regulated while translating.
                if (Math.abs(remaining) <= MOVE_TOLERANCE_MM) {
                    success = true;
                    reason = "Axis target reached";
                    break;
                }

                double signedPower = clamp(remaining * TRANSLATION_KP,
                        -MOVE_MAX_POWER, MOVE_MAX_POWER);
                if (Math.abs(signedPower) < MOVE_MIN_POWER) {
                    signedPower = Math.copySign(MOVE_MIN_POWER, remaining);
                }

                // Transform desired travel vector into current robot axes.
                double delta = Math.toRadians(normalizeAngle(heading - targetHeading));
                double localForward = (strafe ? Math.sin(delta) : Math.cos(delta))
                        * direction * signedPower;
                double localLeft = (strafe ? Math.cos(delta) : -Math.sin(delta))
                        * direction * signedPower;

                // A small corrective left/right velocity in the ORIGINAL
                // field frame. Use it only while reversing. Unlike the old
                // controller, this can never become a 0.33-power strafe.
                double homeLineCorrectionLeft = 0.0;
                if (!strafe && amountMm < 0) {
                    homeLineCorrectionLeft = clamp(
                            -lateralFromHome * BACKWARD_LATERAL_KP,
                            -BACKWARD_LATERAL_MAX, BACKWARD_LATERAL_MAX);
                    // Rotate field-left correction into CURRENT robot axes.
                    localForward += homeLineCorrectionLeft * Math.sin(delta);
                    localLeft += homeLineCorrectionLeft * Math.cos(delta);
                } else if (!strafe && amountMm > 0
                        && ENABLE_FORWARD_LATERAL_CORRECTION) {
                    // Keep outbound behavior unchanged unless explicitly enabled.
                    double correctionLeft = clamp(
                            -lateralFromHome * FORWARD_LATERAL_KP,
                            -FORWARD_LATERAL_MAX, FORWARD_LATERAL_MAX);
                    localForward += correctionLeft * Math.sin(delta);
                    localLeft += correctionLeft * Math.cos(delta);
                }

                double turn = clamp(headingError * MOVE_HEADING_KP,
                        -MOVE_MAX_TURN, MOVE_MAX_TURN);
                // Keep steering corrections from overpowering the last few mm.
                if (Math.abs(remaining) < 50.0) {
                    turn = clamp(turn, -0.07, 0.07);
                }
                driveMecanum(localForward, -localLeft, turn);

                if (Math.abs(remaining) <= bestRemaining - MIN_PROGRESS_MM) {
                    bestRemaining = Math.abs(remaining);
                    progressTimer.reset();
                }
                if (progressTimer.seconds() > STALL_SEC) {
                    reason = String.format(Locale.US,
                            "No axis progress: remaining=%.1f mm, cross=%.1f mm",
                            remaining, crossTrack);
                    break;
                }

                showTime(phase, phaseStart);
                telemetry.addData("Axis", strafe ? "Strafe" : "Forward/back");
                telemetry.addData("Progress mm", "%.1f", progress);
                telemetry.addData("Axis remaining mm", "%.1f", remaining);
                telemetry.addData("Cross track mm", "%.1f", crossTrack);
                telemetry.addData("Lateral from home mm", "%.1f", lateralFromHome);
                telemetry.addData("Home line correction left", "%.3f", homeLineCorrectionLeft);
                telemetry.addData("Target heading deg", "%.2f", targetHeading);
                telemetry.addData("Actual heading deg", "%.2f", heading);
                telemetry.addData("Heading error deg", "%.1f", headingError);
                telemetry.addData("Cmd forward", "%.3f", localForward);
                telemetry.addData("Cmd strafe right", "%.3f", -localLeft);
                telemetry.addData("Cmd turn", "%.3f", turn);
                telemetry.update();
                sleep(CONTROL_PERIOD_MS);
            }
        } finally {
            driveMecanum(0, 0, 0);
        }
        if (!success) lastFailure = reason;
        logStep(phase, reason, phaseStart);
        return success;
    }

    /** Absolute heading. Smooth, damped, and rate-aware for quick return turn. */
    private boolean turnToHeading(String phase, double targetHeading) {
        currentPhase = phase;
        lastFailure = "None";
        double phaseStart = autoTimer.seconds();
        if (!opModeIsActive()) return failStep(phase, "OpMode stopped", phaseStart);
        odo.update();
        if (!pinpointReady()) return failStep(phase, "Pinpoint not READY", phaseStart);

        double prevHeading = odo.getHeading(AngleUnit.DEGREES);
        double prevSec = autoTimer.seconds();
        double filteredRate = 0.0;
        double appliedPower = 0.0;
        double lastProgressHeading = prevHeading;
        ElapsedTime stallTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;
        boolean success = false;
        String reason = "OpMode stopped";

        try {
            while (opModeIsActive()) {
                odo.update();
                if (!pinpointReady()) { reason = "Pinpoint not READY"; break; }
                double heading = odo.getHeading(AngleUnit.DEGREES);
                double now = autoTimer.seconds();
                double dt = Math.max(0.001, now - prevSec);

                // Finite-difference heading rate, with filtering to suppress IMU noise.
                if (dt >= 0.012) {
                    double rawRate = normalizeAngle(heading - prevHeading) / dt;
                    rawRate = clamp(rawRate, -360.0, 360.0);
                    filteredRate = (1.0 - TURN_RATE_FILTER) * filteredRate
                            + TURN_RATE_FILTER * rawRate;
                    prevHeading = heading;
                    prevSec = now;
                }

                double error = normalizeAngle(targetHeading - heading);
                boolean nearTarget = Math.abs(error) <= TURN_TOLERANCE_DEG;
                boolean slowEnough = Math.abs(filteredRate)
                        <= TURN_SETTLE_RATE_DEG_PER_SEC;

                if (nearTarget && slowEnough) {
                    driveMecanum(0, 0, 0);
                    appliedPower = 0.0;
                    if (!settling) {
                        settleTimer.reset();
                        settling = true;
                    }
                    if (settleTimer.seconds() >= TURN_SETTLE_SEC) {
                        success = true;
                        reason = "Heading reached and settled";
                        break;
                    }
                } else {
                    settling = false;
                    // tanh() gives smooth static-friction compensation that
                    // approaches zero near the goal; no fixed 0.18-power jump.
                    double frictionPower = TURN_STATIC_FF
                            * Math.tanh(error / TURN_FF_WIDTH_DEG);
                    double requested = TURN_KP * error + frictionPower
                            - TURN_KD * filteredRate;
                    requested = clamp(requested, -TURN_MAX_POWER, TURN_MAX_POWER);

                    // Limit command jumps to reduce jerk on direction reversal.
                    double step = TURN_SLEW_POWER_PER_SEC * Math.min(dt, 0.06);
                    appliedPower = clamp(requested,
                            appliedPower - step, appliedPower + step);
                    driveMecanum(0.0, 0.0, appliedPower);
                }

                if (Math.abs(normalizeAngle(heading - lastProgressHeading)) >= 0.5) {
                    lastProgressHeading = heading;
                    stallTimer.reset();
                }
                if (!settling && stallTimer.seconds() > STALL_SEC) {
                    reason = "No heading progress";
                    break;
                }

                showTime(phase, phaseStart);
                telemetry.addData("Target heading deg", "%.2f", targetHeading);
                telemetry.addData("Current heading deg", "%.2f", heading);
                telemetry.addData("Heading error deg", "%.2f", error);
                telemetry.addData("Yaw rate deg/s", "%.1f", filteredRate);
                telemetry.addData("Turn power", "%.3f", appliedPower);
                telemetry.addData("Settling", settling);
                telemetry.update();
                sleep(CONTROL_PERIOD_MS);
            }
        } finally {
            driveMecanum(0, 0, 0);
        }
        if (!success) lastFailure = reason;
        logStep(phase, reason, phaseStart);
        return success;
    }

    /** Dynamically limits shooting to preserve the configured return reserve. */
    private void shootWithRemainingTime() {
        currentPhase = "3 Shooting";
        double phaseStart = autoTimer.seconds();
        double shootingSeconds = clamp(AUTO_DURATION_SEC - RETURN_RESERVE_SEC
                - phaseStart, 0.0, MAX_SHOOT_SEC);
        telemetry.log().add(String.format(Locale.US,
                "SHOOT start t=%.2fs, duration=%.2fs, auto remaining=%.2fs",
                phaseStart, shootingSeconds, autoRemaining()));

        if (shootingSeconds > 0 && opModeIsActive()) {
            double shootEnd = phaseStart + shootingSeconds;
            try {
                hogback.setPower(HOGBACK_POWER);
                windmill.setPower(WINDMILL_POWER);
                while (opModeIsActive() && autoTimer.seconds() < shootEnd) {
                    showTime(currentPhase, phaseStart);
                    telemetry.addData("Shoot remaining s", "%.2f",
                            Math.max(0.0, shootEnd - autoTimer.seconds()));
                    telemetry.addData("Return reserve planned s", "%.1f", RETURN_RESERVE_SEC);
                    telemetry.update();
                    sleep(CONTROL_PERIOD_MS);
                }
            } finally {
                hogback.setPower(0);
                windmill.setPower(0);
            }
        }
        logStep(currentPhase, shootingSeconds <= 0 ? "Skipped" : "Stopped", phaseStart);
    }

    /** Publish the same clock fields in EVERY loop (drive, turn, shooting). */
    private void showTime(String phase, double phaseStart) {
        double elapsed = autoTimer.seconds();
        telemetry.addData("Phase", phase);
        telemetry.addData("Auto elapsed s", "%.2f", elapsed);
        telemetry.addData("Auto remaining s", "%.2f", Math.max(0.0,
                AUTO_DURATION_SEC - elapsed));
        telemetry.addData("Phase elapsed s", "%.2f", elapsed - phaseStart);
        if (!Double.isNaN(returnStartSec)) {
            telemetry.addData("Return elapsed s", "%.2f", elapsed - returnStartSec);
        }
        telemetry.addData("Last completed", lastCompletedStep);
    }

    /** Persist each step summary in DS telemetry log; don't lose it on update(). */
    private void logStep(String phase, String outcome, double phaseStart) {
        double elapsed = autoTimer.seconds();
        String entry = String.format(Locale.US,
                "%s: %s | step %.2fs | auto %.2fs | left %.2fs",
                phase, outcome, elapsed - phaseStart, elapsed, autoRemaining());
        telemetry.log().add(entry);
        if (outcome.startsWith("Heading reached") ||
                outcome.startsWith("Axis target") ||
                outcome.startsWith("Stopped") ||
                outcome.startsWith("Skipped")) {
            lastCompletedStep = phase;
        }
        showTime(phase + " END", phaseStart);
        telemetry.addData("Step result", outcome);
        telemetry.update();
    }

    private boolean failStep(String phase, String reason, double phaseStart) {
        lastFailure = reason;
        logStep(phase, reason, phaseStart);
        return false;
    }

    private double autoRemaining() {
        return Math.max(0.0, AUTO_DURATION_SEC - autoTimer.seconds());
    }

    // Standard X-configuration mecanum mixing with existing motor directions.
    // Positive forward = forward; positive strafeRight = RIGHT;
    // positive turnCCW = LEFT / counterclockwise.
    private void driveMecanum(double forward, double strafeRight, double turnCCW) {
        double fl = forward + strafeRight - turnCCW;
        double fr = forward - strafeRight + turnCCW;
        double bl = forward - strafeRight - turnCCW;
        double br = forward + strafeRight + turnCCW;
        double scale = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        leftFront.setPower(fl / scale);
        rightFront.setPower(fr / scale);
        leftBack.setPower(bl / scale);
        rightBack.setPower(br / scale);
    }

    private boolean pinpointReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private double normalizeAngle(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    private void stopEverything() {
        driveMecanum(0, 0, 0);
        hogback.setPower(0);
        windmill.setPower(0);
        intake.stop();
    }
}
