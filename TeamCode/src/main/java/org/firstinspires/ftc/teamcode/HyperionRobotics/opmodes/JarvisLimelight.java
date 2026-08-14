package org.firstinspires.ftc.teamcode.HyperionRobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.HyperionRobot;
import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.HyperionRobotics.limelight.SensorLimelight3A;

/**
 * Full Hyperion autonomous sequence.
 *
 * Sequence:
 *
 *   1. Pipeline 0: find AprilTag 12.
 *   2. Center Tag 12 using Limelight TX.
 *   3. Initialize Pinpoint reference once Tag 12 is centered.
 *   4. Approach using:
 *        - Limelight absolute range
 *        - Pinpoint measured displacement
 *   5. Stop approximately 24 inches from Tag 12.
 *   6. Pipeline 1: switch to yellow color-ball tracking.
 *   7. Scan left to -90 degrees and right to +90 degrees.
 *   8. Track the selected yellow ColorTarget.
 *   9. Run intake while approaching.
 *  10. When the ball drops below the camera around 15-18 inches:
 *        - preserve last estimated range,
 *        - preserve Pinpoint heading,
 *        - use Pinpoint for final straight travel,
 *        - drive the estimated remaining distance + 4 inches,
 *        - command zero turn during final ingestion.
 */
@Autonomous(
        name = "Jarvis Limelight",
        group = "Autonomous"
)
public class JarvisLimelight extends LinearOpMode {

    private HyperionRobot robot;
    private SensorLimelight3A limelight;

    // =====================================================================
    // AprilTag
    // =====================================================================

    private static final int TARGET_TAG_ID = 12;

    private static final double TAG_STOP_DISTANCE_IN = 24.0;
    private static final double TAG_STOP_TOLERANCE_IN = 1.0;

    private static final int REQUIRED_STABLE_TAG_FRAMES = 3;
    private static final double MAX_TAG_APPROACH_TIME_SEC = 20.0;

    private static final double TAG_FAR_POWER = 0.42;
    private static final double TAG_MID_POWER = 0.24;
    private static final double TAG_NEAR_POWER = 0.24;

    private static final double TAG_STEER_KP = 0.020;
    private static final double TAG_MIN_STEER = 0.12;
    private static final double TAG_MAX_STEER = 0.22;

    private static final double TAG_HEADING_LOCK_TX_DEG = 2.0;
    private static final double TAG_HEADING_HOLD_KP = 0.012;
    private static final double TAG_HEADING_MAX_CORRECTION = 0.08;

    /*
     * Search left first.
     */
    private static final double[] TAG_SEARCH_OFFSETS_DEG = {
            0.0,
            -15.0,
            -30.0,
            -45.0,
            -30.0,
            -15.0,
            0.0,
            15.0,
            30.0,
            45.0,
            30.0,
            15.0
    };

    private static final double TAG_SEARCH_KP = 0.012;
    private static final double TAG_SEARCH_MIN_POWER = 0.14;
    private static final double TAG_SEARCH_MAX_POWER = 0.22;
    private static final double TAG_SEARCH_TOLERANCE_DEG = 4.0;
    private static final double TAG_SEARCH_HOLD_SEC = 0.20;

    private static final double MAX_TAG_DROPOUT_CONTINUE_SEC = 0.35;

    // =====================================================================
    // Ball tracking
    // =====================================================================

    private static final double MAX_BALL_COLLECTION_TIME_SEC = 30.0;

    private static final int REQUIRED_STABLE_BALL_FRAMES = 4;
    private static final int REQUIRED_CENTERED_FRAMES_FOR_COMMIT = 4;

    private static final double BALL_STRAIGHT_TX_DEG = 2.5;
    private static final double BALL_CENTERED_TX_DEG = 4.0;
    private static final double BALL_CREEP_TX_DEG = 12.0;

    private static final double BALL_FORWARD_POWER = 0.48;
    private static final double BALL_CREEP_POWER = 0.48;

    private static final double BALL_STEER_KP = 0.018;
    private static final double BALL_MIN_STEER = 0.12;
    private static final double BALL_MAX_STEER = 0.22;

    private static final double BALL_INTAKE_START_TX_DEG = 12.0;

    /*
     * Close-range handoff:
     * once range and bearing are reliable, stop servoing from vision.
     */
    private static final double BALL_PINPOINT_LOCK_DISTANCE_IN = 20.0;
    private static final double BALL_PINPOINT_LOCK_MAX_TX_DEG = 20.0;

    /*
     * Pinpoint-only rotation to the heading calculated at handoff.
     */
    private static final double BALL_LOCK_TURN_KP = 0.014;
    private static final double BALL_LOCK_MIN_TURN_POWER = 0.12;
    private static final double BALL_LOCK_MAX_TURN_POWER = 0.20;
    private static final double BALL_LOCK_HEADING_TOLERANCE_DEG = 2.0;
    private static final int BALL_LOCK_HEADING_STABLE_LOOPS = 3;
    private static final double MAX_BALL_LOCK_TURN_TIME_SEC = 3.0;

    /*
     * Scan both sides to 90 degrees.
     */
    private static final double[] BALL_SEARCH_OFFSETS_DEG = {
            0.0,
            -30.0,
            -60.0,
            -90.0,
            -60.0,
            -30.0,
            0.0,
            30.0,
            60.0,
            90.0,
            60.0,
            30.0
    };

    private static final double BALL_SEARCH_KP = 0.012;
    private static final double BALL_SEARCH_MIN_POWER = 0.14;
    private static final double BALL_SEARCH_MAX_POWER = 0.22;
    private static final double BALL_SEARCH_TOLERANCE_DEG = 5.0;

    private static final double BALL_SEARCH_INITIAL_HOLD_SEC = 0.50;
    private static final double BALL_SEARCH_HOLD_SEC = 0.25;

    private static final double ODOMETRY_TURN_SIGN = -1.0;

    // =====================================================================
    // Ball close-loss / final Pinpoint capture
    // =====================================================================

    /*
     * The ball becomes invisible at roughly 15-18 inches.
     * Permit handoff when the last calculated range was <= this value.
     */
    private static final double BALL_CLOSE_LOSS_MAX_DISTANCE_IN = 24.0;

    private static final double MAX_RECENT_BALL_LOSS_SEC = 0.35;

    /*
     * Physical camera-to-intake correction.
     *
     * Camera is currently almost directly above the intake, so this remains
     * zero for now. Keep it explicit in case the geometry changes later.
     */
    private static final double CAMERA_TO_INTAKE_CAPTURE_OFFSET_IN = 0.0;

    /*
     * Only intentional distance buffer beyond the calculated ball distance.
     */
    private static final double BALL_CAPTURE_OVERTRAVEL_IN = 12.0;

    private static final double FINAL_CAPTURE_POWER = 0.10;
    private static final double FINAL_CAPTURE_TOLERANCE_IN = 0.35;

    /*
     * Safety timeout scales with commanded distance instead of limiting how
     * far Pinpoint is allowed to drive.
     */
    private static final double FINAL_CAPTURE_TIMEOUT_BASE_SEC = 3.0;
    private static final double FINAL_CAPTURE_TIMEOUT_SEC_PER_IN = 0.35;

    // =====================================================================
    // Ball distance calibration
    // =====================================================================

    /*
     * Current measured ball TY calibration.
     *
     * Replace/extend these after you collect current Pipeline-1 measurements
     * at 2, 4, 6 and 8 ft.
     */
    private static final double BALL_TY_18_IN = -19.48;
    private static final double BALL_TY_24_IN = -16.87;
    private static final double BALL_TY_36_IN = -12.53;

    private static final double BALL_DISTANCE_18_IN = 18.0;
    private static final double BALL_DISTANCE_24_IN = 24.0;
    private static final double BALL_DISTANCE_36_IN = 36.0;

    private static final double BALL_DISTANCE_FILTER_ALPHA = 0.30;

    // =====================================================================
    // Main
    // =====================================================================

    @Override
    public void runOpMode() {

        robot =
                new HyperionRobot(
                        hardwareMap
                );

        limelight =
                new SensorLimelight3A(
                        hardwareMap
                );

        telemetry.addData(
                "Status",
                "Initialized"
        );

        telemetry.addData(
                "Sequence",
                "Tag 12 -> 24in -> Yellow Ball"
        );

        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {

            limelight.stop();
            return;
        }

        // -----------------------------------------------------------------
        // STEP 1: APRILTAG
        // -----------------------------------------------------------------

        telemetry.addData(
                "AUTO STEP",
                "1 - Approach AprilTag %d",
                TARGET_TAG_ID
        );

        telemetry.update();

        boolean reachedTag =
                approachAprilTag();

        robot.drive.stop();

        if (!reachedTag
                || !opModeIsActive()) {

            robot.stopAll();
            limelight.stop();

            telemetry.addData(
                    "Status",
                    "STOPPED - AprilTag approach failed"
            );

            telemetry.update();
            sleep(1200);
            return;
        }

        // -----------------------------------------------------------------
        // STEP 2: BALL
        // -----------------------------------------------------------------

        telemetry.addData(
                "AUTO STEP",
                "2 - Tag reached. Scan for yellow ball"
        );

        telemetry.addData(
                "Tag Stop Target",
                "%.1f in",
                TAG_STOP_DISTANCE_IN
        );

        telemetry.update();

        sleep(300);

        boolean collected =
                collectYellowBall();

        robot.stopAll();
        limelight.stop();

        telemetry.addData(
                "Status",
                collected
                        ? "Autonomous complete - ball captured"
                        : "Ball collection timed out"
        );

        telemetry.update();

        sleep(1000);
    }

    // =====================================================================
    // AprilTag approach
    // =====================================================================

    private boolean approachAprilTag() {

        limelight.useAprilTagPipeline();

        /*
         * Give Pipeline 0 time to become active.
         */
        sleep(500);

        robot.updateSensors();
        limelight.update();

        final double searchCenterHeading =
                robot.odometry.getHeadingDeg();

        ElapsedTime approachTimer =
                new ElapsedTime();

        ElapsedTime searchHoldTimer =
                new ElapsedTime();

        ElapsedTime timeSinceTagSeen =
                new ElapsedTime();

        int searchWaypoint =
                0;

        int stableFrames =
                0;

        boolean hasSeenTag =
                false;

        boolean pinpointReferenceValid =
                false;

        boolean headingLockValid =
                false;

        /*
         * Pinpoint reference established once tag is centered.
         */
        double referenceX =
                Double.NaN;

        double referenceY =
                Double.NaN;

        double referenceDistance =
                Double.NaN;

        double lockedHeadingDeg =
                Double.NaN;

        while (opModeIsActive()
                && approachTimer.seconds()
                < MAX_TAG_APPROACH_TIME_SEC) {

            robot.updateSensors();
            limelight.update();

            double currentX =
                    robot.odometry.getX();

            double currentY =
                    robot.odometry.getY();

            double currentHeading =
                    robot.odometry.getHeadingDeg();

            SensorLimelight3A.AprilTagTarget tag =
                    limelight.getAprilTag(
                            TARGET_TAG_ID
                    );

            telemetry.addData(
                    "AUTO STEP",
                    "APRILTAG"
            );

            telemetry.addData(
                    "Active Pipeline",
                    "%d / %s",
                    limelight.getActivePipelineIndex(),
                    limelight.getActivePipelineType()
            );

            telemetry.addData(
                    "Tag " + TARGET_TAG_ID + " Visible",
                    tag != null
            );

            // -------------------------------------------------------------
            // TAG NOT VISIBLE
            // -------------------------------------------------------------

            if (tag == null) {

                stableFrames =
                        0;

                if (!hasSeenTag) {

                    double requestedOffset =
                            TAG_SEARCH_OFFSETS_DEG[
                                    searchWaypoint
                                    ];

                    double requestedHeading =
                            searchCenterHeading
                                    + requestedOffset;

                    double headingError =
                            wrapDegrees(
                                    requestedHeading
                                            - currentHeading
                            );

                    if (Math.abs(
                            headingError
                    ) <= TAG_SEARCH_TOLERANCE_DEG) {

                        robot.drive.stop();

                        if (searchHoldTimer.seconds()
                                >= TAG_SEARCH_HOLD_SEC) {

                            searchWaypoint++;

                            if (searchWaypoint
                                    >= TAG_SEARCH_OFFSETS_DEG.length) {

                                searchWaypoint =
                                        0;
                            }

                            searchHoldTimer.reset();
                        }

                    } else {

                        searchHoldTimer.reset();

                        double magnitude =
                                Range.clip(
                                        Math.abs(
                                                headingError
                                        ) * TAG_SEARCH_KP,
                                        TAG_SEARCH_MIN_POWER,
                                        TAG_SEARCH_MAX_POWER
                                );

                        double turn =
                                ODOMETRY_TURN_SIGN
                                        * Math.signum(
                                        headingError
                                )
                                        * magnitude;

                        robot.drive.arcadeDrive(
                                0.0,
                                turn
                        );
                    }

                    telemetry.addData(
                            "Phase",
                            "Searching LEFT first for Tag %d",
                            TARGET_TAG_ID
                    );

                    telemetry.addData(
                            "Search Offset",
                            "%.1f deg",
                            requestedOffset
                    );

                    telemetry.addData(
                            "Current Heading",
                            "%.1f deg",
                            currentHeading
                    );

                    telemetry.update();
                    idle();
                    continue;
                }

                /*
                 * Brief vision dropout:
                 * continue from Pinpoint only if we already have a valid
                 * centered reference.
                 */
                double lostTime =
                        timeSinceTagSeen.seconds();

                if (pinpointReferenceValid
                        && headingLockValid
                        && lostTime
                        <= MAX_TAG_DROPOUT_CONTINUE_SEC) {

                    double traveled =
                            Math.hypot(
                                    currentX - referenceX,
                                    currentY - referenceY
                            );

                    double pinpointRemaining =
                            referenceDistance
                                    - traveled
                                    - TAG_STOP_DISTANCE_IN;

                    if (pinpointRemaining
                            <= TAG_STOP_TOLERANCE_IN) {

                        robot.drive.stop();

                        telemetry.addData(
                                "Phase",
                                "STOP - Pinpoint reached 24in"
                        );

                        telemetry.update();

                        return true;
                    }

                    double headingError =
                            wrapDegrees(
                                    lockedHeadingDeg
                                            - currentHeading
                            );

                    double headingCorrection =
                            Range.clip(
                                    headingError
                                            * TAG_HEADING_HOLD_KP,
                                    -TAG_HEADING_MAX_CORRECTION,
                                    TAG_HEADING_MAX_CORRECTION
                            );

                    double drive =
                            Range.clip(
                                    pinpointRemaining
                                            * RobotConstants.LL_RANGE_KP,
                                    0.0,
                                    TAG_NEAR_POWER
                            );

                    robot.drive.arcadeDrive(
                            drive,
                            headingCorrection
                    );

                    telemetry.addData(
                            "Phase",
                            "Tag dropout - Pinpoint continuation"
                    );

                    telemetry.addData(
                            "Pinpoint Remaining",
                            "%.2f in",
                            pinpointRemaining
                    );

                    telemetry.update();
                    idle();
                    continue;
                }

                /*
                 * Outside the short dropout window, do not continue forward.
                 */
                robot.drive.stop();

                telemetry.addData(
                        "Phase",
                        "Tag lost - stopping to reacquire"
                );

                telemetry.update();
                idle();
                continue;
            }

            // -------------------------------------------------------------
            // TAG VISIBLE
            // -------------------------------------------------------------

            hasSeenTag =
                    true;

            timeSinceTagSeen.reset();
            searchHoldTimer.reset();

            double cameraDistance =
                    tag.getDistanceInches();

            /*
             * CRITICAL SAFETY CHECK:
             *
             * A bad/zero Limelight pose previously produced a 0.00-inch
             * distance, which made Jarvis immediately believe it had already
             * reached the 24-inch AprilTag stop point.
             *
             * Do not allow an invalid or implausible range to complete the
             * AprilTag phase.
             */
            if (!Double.isFinite(cameraDistance)
                    || cameraDistance < 6.0
                    || cameraDistance > 240.0) {

                robot.drive.stop();

                telemetry.addData(
                        "Phase",
                        "Tag visible - invalid range"
                );

                telemetry.addData(
                        "Camera Distance",
                        "%.2f in",
                        cameraDistance
                );

                telemetry.addData(
                        "Tag TA",
                        "%.4f%%",
                        tag.area
                );

                telemetry.update();
                idle();
                continue;
            }

            stableFrames++;

            if (stableFrames
                    < REQUIRED_STABLE_TAG_FRAMES) {

                robot.drive.stop();

                telemetry.addData(
                        "Phase",
                        "Validating Tag %d/%d",
                        stableFrames,
                        REQUIRED_STABLE_TAG_FRAMES
                );

                telemetry.addData(
                        "Camera Distance",
                        "%.2f in",
                        cameraDistance
                );

                telemetry.update();
                idle();
                continue;
            }

            double visionRemaining =
                    cameraDistance
                            - TAG_STOP_DISTANCE_IN;

            /*
             * Vision independently says STOP.
             */
            if (visionRemaining
                    <= TAG_STOP_TOLERANCE_IN) {

                robot.drive.stop();

                telemetry.addData(
                        "Phase",
                        "STOP - Vision reached 24in"
                );

                telemetry.addData(
                        "Camera Distance",
                        "%.2f in",
                        cameraDistance
                );

                telemetry.update();

                return true;
            }

            /*
             * Establish Pinpoint reference only after the tag is nearly
             * centered.
             */
            if (!pinpointReferenceValid
                    && Math.abs(
                    tag.tx
            ) <= TAG_HEADING_LOCK_TX_DEG) {

                referenceX =
                        currentX;

                referenceY =
                        currentY;

                referenceDistance =
                        cameraDistance;

                lockedHeadingDeg =
                        currentHeading;

                pinpointReferenceValid =
                        true;

                headingLockValid =
                        true;
            }

            /*
             * Refresh the heading lock every time vision confirms that the tag
             * is centered.
             */
            if (Math.abs(
                    tag.tx
            ) <= TAG_HEADING_LOCK_TX_DEG) {

                lockedHeadingDeg =
                        currentHeading;

                headingLockValid =
                        true;
            }

            double pinpointRemaining =
                    Double.NaN;

            if (pinpointReferenceValid) {

                double traveled =
                        Math.hypot(
                                currentX - referenceX,
                                currentY - referenceY
                        );

                pinpointRemaining =
                        referenceDistance
                                - traveled
                                - TAG_STOP_DISTANCE_IN;

                /*
                 * Pinpoint independently says STOP.
                 */
                if (pinpointRemaining
                        <= TAG_STOP_TOLERANCE_IN) {

                    robot.drive.stop();

                    telemetry.addData(
                            "Phase",
                            "STOP - Pinpoint reached 24in"
                    );

                    telemetry.addData(
                            "Camera Distance",
                            "%.2f in",
                            cameraDistance
                    );

                    telemetry.addData(
                            "Pinpoint Remaining",
                            "%.2f in",
                            pinpointRemaining
                    );

                    telemetry.update();

                    return true;
                }
            }

            /*
             * Use the more conservative remaining distance for speed control.
             */
            double controlRemaining =
                    visionRemaining;

            if (Double.isFinite(
                    pinpointRemaining
            )) {

                controlRemaining =
                        Math.min(
                                visionRemaining,
                                pinpointRemaining
                        );
            }

            controlRemaining =
                    Math.max(
                            0.0,
                            controlRemaining
                    );

            /*
             * Limelight TX controls tag centering.
             */
            double visionSteer =
                    Range.clip(
                            tag.tx
                                    * TAG_STEER_KP,
                            -TAG_MAX_STEER,
                            TAG_MAX_STEER
                    );

            if (Math.abs(
                    tag.tx
            ) > RobotConstants.LL_TX_TOLERANCE_DEG) {

                visionSteer =
                        applyMinimumMagnitude(
                                visionSteer,
                                TAG_MIN_STEER
                        );

            } else {

                visionSteer =
                        0.0;
            }

            /*
             * Pinpoint holds the most recently centered heading.
             */
            double headingCorrection =
                    0.0;

            if (headingLockValid) {

                double headingError =
                        wrapDegrees(
                                lockedHeadingDeg
                                        - currentHeading
                        );

                headingCorrection =
                        Range.clip(
                                headingError
                                        * TAG_HEADING_HOLD_KP,
                                -TAG_HEADING_MAX_CORRECTION,
                                TAG_HEADING_MAX_CORRECTION
                        );
            }

            double steer =
                    Range.clip(
                            visionSteer
                                    + headingCorrection,
                            -TAG_MAX_STEER,
                            TAG_MAX_STEER
                    );

            double speedLimit;

            if (controlRemaining
                    > 30.0) {

                speedLimit =
                        TAG_FAR_POWER;

            } else if (controlRemaining
                    > 10.0) {

                speedLimit =
                        TAG_MID_POWER;

            } else {

                speedLimit =
                        TAG_NEAR_POWER;
            }

            double drive =
                    Range.clip(
                            controlRemaining
                                    * RobotConstants.LL_RANGE_KP,
                            0.0,
                            speedLimit
                    );

            /*
             * Turn first if tag is significantly off-center.
             */
            if (Math.abs(
                    tag.tx
            ) > 10.0) {

                drive =
                        0.0;

            } else if (Math.abs(
                    tag.tx
            ) > 5.0) {

                drive =
                        Math.min(
                                drive,
                                TAG_NEAR_POWER
                        );
            }

            robot.drive.arcadeDrive(
                    drive,
                    steer
            );

            telemetry.addData(
                    "Phase",
                    "APPROACH TAG - Vision + Pinpoint"
            );

            telemetry.addData(
                    "Camera Distance",
                    "%.2f in",
                    cameraDistance
            );

            telemetry.addData(
                    "Vision Remaining",
                    "%.2f in",
                    visionRemaining
            );

            telemetry.addData(
                    "Pinpoint Remaining",
                    Double.isFinite(
                            pinpointRemaining
                    )
                            ? String.format(
                            "%.2f in",
                            pinpointRemaining
                    )
                            : "not initialized"
            );

            telemetry.addData(
                    "Control Remaining",
                    "%.2f in",
                    controlRemaining
            );

            telemetry.addData(
                    "Tag TX",
                    "%.2f deg",
                    tag.tx
            );

            telemetry.addData(
                    "Drive / Steer",
                    "%.2f / %.2f",
                    drive,
                    steer
            );

            telemetry.update();
            idle();
        }

        robot.drive.stop();

        telemetry.addData(
                "Phase",
                "AprilTag approach timed out"
        );

        telemetry.update();

        return false;
    }

    // =====================================================================
    // Ball collection
    // =====================================================================

    private boolean collectYellowBall() {

        limelight.useGamePiecePipeline();
        sleep(500);

        robot.updateSensors();
        limelight.update();

        final double searchCenterHeading =
                robot.odometry.getHeadingDeg();

        final int VISION_TRACKING = 0;
        final int TURN_TO_LOCKED_HEADING = 1;
        final int FINAL_STRAIGHT_CAPTURE = 2;

        int captureState = VISION_TRACKING;
        int searchWaypoint = 0;
        int stableBallFrames = 0;
        int lockedHeadingStableLoops = 0;

        double filteredBallDistance = Double.NaN;

        double lockedBallTx = Double.NaN;
        double lockedBallTy = Double.NaN;
        double lockedBallArea = Double.NaN;
        double lockedBallDistance = Double.NaN;
        double lockedBallHeading = Double.NaN;

        double finalStartX = Double.NaN;
        double finalStartY = Double.NaN;
        double finalDriveDistance = Double.NaN;

        ElapsedTime collectionTimer = new ElapsedTime();
        ElapsedTime waypointHoldTimer = new ElapsedTime();
        ElapsedTime lockedTurnTimer = new ElapsedTime();
        ElapsedTime finalCaptureTimer = new ElapsedTime();

        robot.intake.stop();

        while (opModeIsActive()
                && collectionTimer.seconds() < MAX_BALL_COLLECTION_TIME_SEC) {

            robot.updateSensors();
            limelight.update();

            telemetry.addData("AUTO STEP", "BALL");

            // =============================================================
            // CLOSE-RANGE STATE 1:
            // Turn once to the heading calculated from the last good
            // close-range Limelight observation. Vision is ignored here.
            // =============================================================
            if (captureState == TURN_TO_LOCKED_HEADING) {

                robot.intake.intake();

                double currentHeading =
                        robot.odometry.getHeadingDeg();

                double headingError =
                        wrapDegrees(
                                lockedBallHeading - currentHeading
                        );

                telemetry.addData(
                        "Phase",
                        "PINPOINT TURN TO LOCKED BALL HEADING"
                );
                telemetry.addData(
                        "Locked TX / TY / TA",
                        "%.2f / %.2f / %.4f",
                        lockedBallTx,
                        lockedBallTy,
                        lockedBallArea
                );
                telemetry.addData(
                        "Locked Ball Distance",
                        "%.2f in",
                        lockedBallDistance
                );
                telemetry.addData(
                        "Locked / Current Heading",
                        "%.2f / %.2f deg",
                        lockedBallHeading,
                        currentHeading
                );
                telemetry.addData(
                        "Heading Error",
                        "%.2f deg",
                        headingError
                );

                if (Math.abs(headingError)
                        <= BALL_LOCK_HEADING_TOLERANCE_DEG) {

                    robot.drive.stop();
                    lockedHeadingStableLoops++;

                    telemetry.addData(
                            "Heading Stable",
                            "%d / %d",
                            lockedHeadingStableLoops,
                            BALL_LOCK_HEADING_STABLE_LOOPS
                    );

                    if (lockedHeadingStableLoops
                            >= BALL_LOCK_HEADING_STABLE_LOOPS) {

                        /*
                         * Begin measuring final straight travel only after
                         * rotation is complete.
                         */
                        robot.updateSensors();

                        finalStartX =
                                robot.odometry.getX();

                        finalStartY =
                                robot.odometry.getY();

                        finalDriveDistance =
                                lockedBallDistance
                                        + CAMERA_TO_INTAKE_CAPTURE_OFFSET_IN
                                        + BALL_CAPTURE_OVERTRAVEL_IN;

                        finalCaptureTimer.reset();
                        captureState = FINAL_STRAIGHT_CAPTURE;

                        telemetry.addData(
                                "Transition",
                                "Heading locked -> straight capture"
                        );
                        telemetry.addData(
                                "Final Drive Target",
                                "%.2f in",
                                finalDriveDistance
                        );
                    }

                    telemetry.update();
                    idle();
                    continue;
                }

                lockedHeadingStableLoops = 0;

                if (lockedTurnTimer.seconds()
                        >= MAX_BALL_LOCK_TURN_TIME_SEC) {

                    robot.drive.stop();

                    telemetry.addData(
                            "Capture",
                            "Timed out turning to locked ball heading"
                    );
                    telemetry.update();

                    return false;
                }

                double turnMagnitude =
                        Range.clip(
                                Math.abs(headingError) * BALL_LOCK_TURN_KP,
                                BALL_LOCK_MIN_TURN_POWER,
                                BALL_LOCK_MAX_TURN_POWER
                        );

                double turn =
                        ODOMETRY_TURN_SIGN
                                * Math.signum(headingError)
                                * turnMagnitude;

                robot.drive.arcadeDrive(
                        0.0,
                        turn
                );

                telemetry.addData(
                        "Turn Command",
                        "%.2f",
                        turn
                );

                telemetry.update();
                idle();
                continue;
            }

            // =============================================================
            // CLOSE-RANGE STATE 2:
            // Pinpoint measures distance.  Drive command is deliberately
            // straight: turn = 0.  Vision is ignored.
            // =============================================================
            if (captureState == FINAL_STRAIGHT_CAPTURE) {

                robot.intake.intake();

                double currentX =
                        robot.odometry.getX();

                double currentY =
                        robot.odometry.getY();

                double currentHeading =
                        robot.odometry.getHeadingDeg();

                double progress =
                        Math.hypot(
                                currentX - finalStartX,
                                currentY - finalStartY
                        );

                double remaining =
                        finalDriveDistance - progress;

                telemetry.addData(
                        "Phase",
                        "FINAL CAPTURE - PINPOINT STRAIGHT"
                );
                telemetry.addData(
                        "Locked Ball Distance",
                        "%.2f in",
                        lockedBallDistance
                );
                telemetry.addData(
                        "Camera->Intake Offset",
                        "%.1f in",
                        CAMERA_TO_INTAKE_CAPTURE_OFFSET_IN
                );

                telemetry.addData(
                        "Overtravel",
                        "%.1f in",
                        BALL_CAPTURE_OVERTRAVEL_IN
                );
                telemetry.addData(
                        "Final Drive Target",
                        "%.2f in",
                        finalDriveDistance
                );
                telemetry.addData(
                        "Pinpoint Progress",
                        "%.2f in",
                        progress
                );
                telemetry.addData(
                        "Pinpoint Remaining",
                        "%.2f in",
                        remaining
                );
                telemetry.addData(
                        "Locked / Current Heading",
                        "%.1f / %.1f deg",
                        lockedBallHeading,
                        currentHeading
                );
                telemetry.addData(
                        "Final Turn Command",
                        "0.00"
                );

                if (remaining <= FINAL_CAPTURE_TOLERANCE_IN) {

                    robot.drive.stop();

                    /*
                     * Keep the intake spinning briefly to pull the ball fully
                     * into the mechanism.
                     */
                    sleep(400);

                    telemetry.addData(
                            "Capture",
                            "Pinpoint straight capture completed"
                    );
                    telemetry.update();

                    return true;
                }

                double allowedCaptureTimeSec =
                        FINAL_CAPTURE_TIMEOUT_BASE_SEC
                                + finalDriveDistance
                                * FINAL_CAPTURE_TIMEOUT_SEC_PER_IN;

                if (finalCaptureTimer.seconds()
                        >= allowedCaptureTimeSec) {

                    robot.drive.stop();

                    telemetry.addData(
                            "Capture",
                            "Final capture timed out"
                    );

                    telemetry.addData(
                            "Allowed Capture Time",
                            "%.1f sec",
                            allowedCaptureTimeSec
                    );

                    telemetry.update();

                    return false;
                }

                /*
                 * No Limelight steering and no heading correction here.
                 */
                robot.drive.arcadeDrive(
                        FINAL_CAPTURE_POWER,
                        0.0
                );

                telemetry.addData(
                        "Final Drive / Turn",
                        "%.2f / 0.00",
                        FINAL_CAPTURE_POWER
                );

                telemetry.update();
                idle();
                continue;
            }

            // =============================================================
            // NORMAL VISION TRACKING
            // =============================================================
            SensorLimelight3A.ColorTarget ball =
                    limelight.getBestColorTarget();

            boolean ballVisible =
                    ball != null
                            && limelight.isFresh(
                            RobotConstants.LL_MAX_RESULT_AGE_MS
                    );

            telemetry.addData(
                    "LL Raw Valid",
                    limelight.isValid()
            );
            telemetry.addData(
                    "LL Staleness",
                    "%d ms",
                    limelight.getStalenessMs()
            );
            telemetry.addData(
                    "Color Result Count",
                    limelight.getColorTargets().size()
            );
            telemetry.addData(
                    "Ball Target",
                    ballVisible
            );

            if (ballVisible) {

                waypointHoldTimer.reset();
                stableBallFrames++;

                double rawDistance =
                        estimateBallDistanceFromTy(
                                ball.ty
                        );

                if (Double.isFinite(rawDistance)) {

                    if (!Double.isFinite(filteredBallDistance)) {

                        filteredBallDistance =
                                rawDistance;

                    } else {

                        filteredBallDistance =
                                BALL_DISTANCE_FILTER_ALPHA * rawDistance
                                        + (1.0
                                        - BALL_DISTANCE_FILTER_ALPHA)
                                        * filteredBallDistance;
                    }
                }

                /*
                 * Start intake early.
                 */
                if (Math.abs(ball.tx)
                        <= BALL_INTAKE_START_TX_DEG) {

                    robot.intake.intake();

                } else {

                    robot.intake.stop();
                }

                /*
                 * Require a few stable target frames before either driving or
                 * committing to Pinpoint.
                 */
                if (stableBallFrames
                        < REQUIRED_STABLE_BALL_FRAMES) {

                    robot.drive.stop();

                    telemetry.addData(
                            "Phase",
                            "Confirming yellow ball"
                    );

                    addBallTelemetry(
                            ball,
                            filteredBallDistance,
                            stableBallFrames,
                            0
                    );

                    telemetry.update();
                    idle();
                    continue;
                }

                /*
                 * CLOSE-RANGE LOCK:
                 *
                 * Do not wait for the ball to disappear. At <= 20 inches,
                 * if its bearing is within +/-20 degrees, freeze the current
                 * vision solution and switch completely to Pinpoint.
                 */
                boolean closeEnoughToLock =
                        Double.isFinite(filteredBallDistance)
                                && filteredBallDistance
                                <= BALL_PINPOINT_LOCK_DISTANCE_IN;

                boolean bearingUsable =
                        Math.abs(ball.tx)
                                <= BALL_PINPOINT_LOCK_MAX_TX_DEG;

                if (closeEnoughToLock
                        && bearingUsable) {

                    robot.drive.stop();
                    robot.intake.intake();

                    robot.updateSensors();

                    double currentHeading =
                            robot.odometry.getHeadingDeg();

                    lockedBallTx =
                            ball.tx;

                    lockedBallTy =
                            ball.ty;

                    lockedBallArea =
                            ball.area;

                    lockedBallDistance =
                            filteredBallDistance;

                    /*
                     * Existing robot convention:
                     *
                     * Positive Limelight TX creates a positive arcade-turn
                     * command. Positive arcade turn decreases Pinpoint heading,
                     * so convert the camera bearing using ODOMETRY_TURN_SIGN.
                     */
                    lockedBallHeading =
                            wrapDegrees(
                                    currentHeading
                                            + ODOMETRY_TURN_SIGN
                                            * lockedBallTx
                            );

                    lockedHeadingStableLoops = 0;
                    lockedTurnTimer.reset();
                    captureState = TURN_TO_LOCKED_HEADING;

                    telemetry.addData(
                            "Phase",
                            "LOCK BALL -> PINPOINT"
                    );
                    telemetry.addData(
                            "Locked TX / TY / TA",
                            "%.2f / %.2f / %.4f",
                            lockedBallTx,
                            lockedBallTy,
                            lockedBallArea
                    );
                    telemetry.addData(
                            "Locked Ball Distance",
                            "%.2f in",
                            lockedBallDistance
                    );
                    telemetry.addData(
                            "Current Heading",
                            "%.2f deg",
                            currentHeading
                    );
                    telemetry.addData(
                            "Locked Ball Heading",
                            "%.2f deg",
                            lockedBallHeading
                    );
                    telemetry.addData(
                            "Planned Straight Distance",
                            "%.2f in",
                            lockedBallDistance
                                    + CAMERA_TO_INTAKE_CAPTURE_OFFSET_IN
                                    + BALL_CAPTURE_OVERTRAVEL_IN
                    );

                    telemetry.update();
                    idle();
                    continue;
                }

                /*
                 * Long-/mid-range Limelight tracking.
                 */
                double absTx =
                        Math.abs(ball.tx);

                double drive;
                double steer;

                if (absTx
                        <= BALL_STRAIGHT_TX_DEG) {

                    drive = BALL_FORWARD_POWER;
                    steer = 0.0;

                } else if (absTx
                        <= BALL_CENTERED_TX_DEG) {

                    drive = BALL_FORWARD_POWER;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -0.08,
                                    0.08
                            );

                } else if (absTx
                        <= BALL_CREEP_TX_DEG) {

                    drive = BALL_CREEP_POWER;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -BALL_MAX_STEER,
                                    BALL_MAX_STEER
                            );

                    steer =
                            applyMinimumMagnitude(
                                    steer,
                                    BALL_MIN_STEER
                            );

                } else {

                    /*
                     * Large TX error: rotate first.
                     */
                    drive = 0.0;

                    steer =
                            Range.clip(
                                    ball.tx * BALL_STEER_KP,
                                    -BALL_MAX_STEER,
                                    BALL_MAX_STEER
                            );

                    steer =
                            applyMinimumMagnitude(
                                    steer,
                                    BALL_MIN_STEER
                            );
                }

                robot.drive.arcadeDrive(
                        drive,
                        steer
                );

                telemetry.addData(
                        "Phase",
                        "Tracking yellow ball"
                );

                addBallTelemetry(
                        ball,
                        filteredBallDistance,
                        stableBallFrames,
                        0
                );

                telemetry.addData(
                        "Drive / Steer",
                        "%.2f / %.2f",
                        drive,
                        steer
                );

                telemetry.addData(
                        "Intake Running",
                        Math.abs(ball.tx)
                                <= BALL_INTAKE_START_TX_DEG
                );

                telemetry.update();
                idle();
                continue;
            }

            // =============================================================
            // NO BALL VISIBLE: SEARCH
            // =============================================================
            stableBallFrames = 0;
            filteredBallDistance = Double.NaN;

            robot.intake.stop();

            double requestedOffset =
                    BALL_SEARCH_OFFSETS_DEG[
                            searchWaypoint
                            ];

            double requestedHeading =
                    searchCenterHeading
                            + requestedOffset;

            double currentHeading =
                    robot.odometry.getHeadingDeg();

            double headingError =
                    wrapDegrees(
                            requestedHeading
                                    - currentHeading
                    );

            double holdTime =
                    searchWaypoint == 0
                            ? BALL_SEARCH_INITIAL_HOLD_SEC
                            : BALL_SEARCH_HOLD_SEC;

            if (Math.abs(headingError)
                    <= BALL_SEARCH_TOLERANCE_DEG) {

                robot.drive.stop();

                if (waypointHoldTimer.seconds()
                        >= holdTime) {

                    searchWaypoint++;

                    if (searchWaypoint
                            >= BALL_SEARCH_OFFSETS_DEG.length) {

                        searchWaypoint = 0;
                    }

                    waypointHoldTimer.reset();
                }

            } else {

                waypointHoldTimer.reset();

                double magnitude =
                        Range.clip(
                                Math.abs(headingError)
                                        * BALL_SEARCH_KP,
                                BALL_SEARCH_MIN_POWER,
                                BALL_SEARCH_MAX_POWER
                        );

                double turn =
                        ODOMETRY_TURN_SIGN
                                * Math.signum(headingError)
                                * magnitude;

                robot.drive.arcadeDrive(
                        0.0,
                        turn
                );
            }

            telemetry.addData(
                    "Phase",
                    "Searching LEFT/RIGHT for yellow ball"
            );
            telemetry.addData(
                    "Search Offset",
                    "%.1f deg",
                    requestedOffset
            );
            telemetry.addData(
                    "Current Heading",
                    "%.1f deg",
                    currentHeading
            );
            telemetry.addData(
                    "Heading Error",
                    "%.1f deg",
                    headingError
            );

            telemetry.update();
            idle();
        }

        robot.drive.stop();
        robot.intake.stop();

        return false;
    }

    // =====================================================================
    // Ball distance
    // =====================================================================

    private double estimateBallDistanceFromTy(
            double ty
    ) {

        if (!Double.isFinite(
                ty
        )) {

            return Double.NaN;
        }

        /*
         * 18-24 inch region.
         */
        if (ty
                <= BALL_TY_24_IN) {

            return linearInterpolate(
                    ty,
                    BALL_TY_18_IN,
                    BALL_DISTANCE_18_IN,
                    BALL_TY_24_IN,
                    BALL_DISTANCE_24_IN
            );
        }

        /*
         * 24-36 inch region; extrapolated farther away until new
         * 48/72/96-inch calibration is collected.
         */
        return linearInterpolate(
                ty,
                BALL_TY_24_IN,
                BALL_DISTANCE_24_IN,
                BALL_TY_36_IN,
                BALL_DISTANCE_36_IN
        );
    }

    private double linearInterpolate(
            double x,
            double x1,
            double y1,
            double x2,
            double y2
    ) {

        if (Math.abs(
                x2 - x1
        ) < 1e-9) {

            return Double.NaN;
        }

        double fraction =
                (x - x1)
                        / (x2 - x1);

        return y1
                + fraction
                * (y2 - y1);
    }

    // =====================================================================
    // Helpers
    // =====================================================================

    private void addBallTelemetry(
            SensorLimelight3A.ColorTarget ball,
            double distance,
            int stableFrames,
            int centeredFrames
    ) {

        telemetry.addData(
                "Ball TX / TY / TA",
                "%.2f / %.2f / %.4f",
                ball.tx,
                ball.ty,
                ball.area
        );

        telemetry.addData(
                "Calculated Ball Distance",
                Double.isFinite(
                        distance
                )
                        ? String.format(
                        "%.2f in",
                        distance
                )
                        : "n/a"
        );

        telemetry.addData(
                "Stable Frames",
                "%d / %d",
                stableFrames,
                REQUIRED_STABLE_BALL_FRAMES
        );

        telemetry.addData(
                "Centered Frames",
                "%d / %d",
                centeredFrames,
                REQUIRED_CENTERED_FRAMES_FOR_COMMIT
        );

        telemetry.addData(
                "Active Pipeline",
                "%d / %s",
                limelight.getActivePipelineIndex(),
                limelight.getActivePipelineType()
        );
    }

    private double applyMinimumMagnitude(
            double value,
            double minimumMagnitude
    ) {

        if (Math.abs(
                value
        ) < 1e-9) {

            return 0.0;
        }

        if (Math.abs(
                value
        ) < minimumMagnitude) {

            return Math.copySign(
                    minimumMagnitude,
                    value
            );
        }

        return value;
    }

    private double wrapDegrees(
            double degrees
    ) {

        while (degrees
                > 180.0) {

            degrees -=
                    360.0;
        }

        while (degrees
                <= -180.0) {

            degrees +=
                    360.0;
        }

        return degrees;
    }
}