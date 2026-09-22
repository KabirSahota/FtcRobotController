package org.firstinspires.ftc.teamcode.HyperionRobotics.constants;

/**
 * HyperionRobotics — hardware map names and tunable constants.
 */
public final class RobotConstants {

    private RobotConstants() {}

    // ---- Drivetrain Configuration ----

    public enum DriveType {
        TANK_2_MOTOR,
        TANK_4_MOTOR,
        TANK_6_MOTOR,
        MECANUM
    }

    public static final DriveType ACTIVE_DRIVE_TYPE =
            DriveType.TANK_2_MOTOR;

    // ---- Drive Motors ----

    public static final String LEFT_FRONT = "leftFront";
    public static final String LEFT_MIDDLE = "leftMiddle";
    public static final String LEFT_BACK = "leftBack";

    public static final String RIGHT_FRONT = "rightFront";
    public static final String RIGHT_MIDDLE = "rightMiddle";
    public static final String RIGHT_BACK = "rightBack";

    public static final double DRIVE_TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;
    public static final double DRIVE_GEAR_RATIO = 1.0;

    public static final double COUNTS_PER_INCH =
            (DRIVE_TICKS_PER_REV * DRIVE_GEAR_RATIO)
                    / (Math.PI * WHEEL_DIAMETER_IN);

    public static final double TELEOP_DRIVE_SCALE = 1.0;
    public static final double TELEOP_TURN_SCALE = 0.75;

    public static final double AUTO_DRIVE_POWER = 0.5;
    public static final double AUTO_TURN_POWER = 0.4;

    // ---- Intake ----

    public static final String INTAKE_MOTOR = "intakeMotor";

    public static final String INTAKE_SERVO_LEFT =
            "intakeServoLeft";

    public static final String INTAKE_SERVO_RIGHT =
            "intakeServoRight";

    public static final double INTAKE_IN_POWER = 1.00;
    public static final double INTAKE_OUT_POWER = -0.70;

    public static final double INTAKE_SERVO_OPEN = 0.15;
    public static final double INTAKE_SERVO_CLOSED = 0.85;

    // ---- Viper ----

    public static final String VIPER_MOTOR = "viperMotor";
    public static final String VIPER_MOTOR_2 = "viperMotor2";

    public static final boolean VIPER_DUAL_MOTOR = false;

    public static final int VIPER_STAGE_0 = 0;
    public static final int VIPER_STAGE_1 = 900;
    public static final int VIPER_STAGE_2 = 1800;
    public static final int VIPER_STAGE_3 = 2700;
    public static final int VIPER_STAGE_4 = 3600;

    public static final int VIPER_MAX_TICKS = 3800;
    public static final double VIPER_POWER = 0.90;
    public static final int VIPER_TOLERANCE = 25;

    // ---- Pinpoint ----
//used to be pinpoint
    public static final String PINPOINT = "odo";

    public static final double PINPOINT_X_OFFSET_MM = 100.0;
    public static final double PINPOINT_Y_OFFSET_MM = -40.0;

    // ---- Limelight 3A ----

    public static final String LIMELIGHT = "limelight";

    /*
     * Pipeline layout:
     *
     * 0 = AprilTag
     * 1 = Neural Detector (game piece)
     * 2 = Driver / spare
     */
    public static final int LL_PIPELINE_APRILTAG = 0;
    public static final int LL_PIPELINE_GAMEPIECE = 1;
    public static final int LL_PIPELINE_DRIVER = 2;

    /*
     * KEEP THIS BLANK FOR THE FIRST TEST.
     *
     * Run "Limelight Neural Detector Test" while facing the ball.
     * Read:
     *
     *   Classes
     *   Detector 0 Class
     *   Detector 1 Class
     *   ...
     *
     * Then replace "" with the EXACT class returned for the ball.
     *
     * Do not guess the class name.
     */
    public static final String GAME_PIECE_CLASS_NAME = "";

    /*
     * Freshness threshold for normal robot control.
     */
    public static final long LL_MAX_RESULT_AGE_MS = 150;

    public static final double LL_TX_TOLERANCE_DEG = 1.5;

    /*
     * Retained only for compatibility with older code.
     * The neural detector abstraction does NOT use this to reject targets.
     */
    public static final double LL_TA_MIN = 0.15;

    public static final double LL_AIM_KP = 0.025;
    public static final double LL_RANGE_KP = 0.04;

    public static final double LL_DESIRED_TAG_DISTANCE_IN = 12.0;

    public static final double LL_MAX_AIM_POWER = 0.35;
    public static final double LL_MAX_DRIVE_POWER = 0.40;

    // ---- Preset Field Poses ----

    public static final double FIELD_SCORING_X = 48.0;
    public static final double FIELD_SCORING_Y = 0.0;

    public static final double FIELD_HANG_X = 0.0;
    public static final double FIELD_HANG_Y = 60.0;

    public static final double FIELD_PRESET_X = 36.0;
    public static final double FIELD_PRESET_Y = 36.0;
}