package org.firstinspires.ftc.teamcode.HyperionRobotics.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HyperionRobotics.constants.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Universal Drivetrain Handler.
 * Supports 2-motor Tank, 4-motor Tank, 6-motor Tank, and 4-motor Mecanum (Strafe).
 */
public class DriveTrain {
    private final DcMotorEx leftFront, leftMiddle, leftBack;
    private final DcMotorEx rightFront, rightMiddle, rightBack;
    
    private final List<DcMotorEx> leftMotors = new ArrayList<>();
    private final List<DcMotorEx> rightMotors = new ArrayList<>();
    private final List<DcMotorEx> allMotors = new ArrayList<>();

    private final RobotConstants.DriveType type;

    public DriveTrain(HardwareMap hardwareMap) {
        this.type = RobotConstants.ACTIVE_DRIVE_TYPE;

        // Initialize all possible motor slots
        leftFront   = safeGet(hardwareMap, RobotConstants.LEFT_FRONT);
        leftBack    = safeGet(hardwareMap, RobotConstants.LEFT_BACK);
        rightFront  = safeGet(hardwareMap, RobotConstants.RIGHT_FRONT);
        rightBack   = safeGet(hardwareMap, RobotConstants.RIGHT_BACK);

        rightMiddle = safeGet(hardwareMap, RobotConstants.RIGHT_MIDDLE);
        leftMiddle  = safeGet(hardwareMap, RobotConstants.LEFT_MIDDLE);


        // Organize motors based on drivetrain type
        configureGroups();

        // Set directions: Inverted to fix the robot moving backwards
        for (DcMotorEx m : leftMotors)  m.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : rightMotors) m.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configureGroups() {
        switch (type) {
            case TANK_2_MOTOR:
                if (leftBack != null) leftMotors.add(leftBack);
                if (rightBack != null) rightMotors.add(rightBack);
                break;
            case TANK_4_MOTOR:
            case MECANUM:
                if (leftFront != null) leftMotors.add(leftFront);
                if (leftBack != null) leftMotors.add(leftBack);
                if (rightFront != null) rightMotors.add(rightFront);
                if (rightBack != null) rightMotors.add(rightBack);
                break;
            case TANK_6_MOTOR:
                if (leftFront != null) leftMotors.add(leftFront);
                if (leftMiddle != null) leftMotors.add(leftMiddle);
                if (leftBack != null) leftMotors.add(leftBack);
                if (rightFront != null) rightMotors.add(rightFront);
                if (rightMiddle != null) rightMotors.add(rightMiddle);
                if (rightBack != null) rightMotors.add(rightBack);
                break;
        }
        allMotors.addAll(leftMotors);
        allMotors.addAll(rightMotors);
    }

    private DcMotorEx safeGet(HardwareMap hw, String name) {
        try { return hw.get(DcMotorEx.class, name); } 
        catch (Exception e) { return null; }
    }

    /** Standard Arcade Drive (Drive + Turn) */
    public void arcadeDrive(double drive, double turn) {
        arcadeDrive(drive, turn, 0.0);
    }

    /** Extended Arcade Drive with Strafe (Mecanum only) */
    public void arcadeDrive(double drive, double turn, double strafe) {
        if (type == RobotConstants.DriveType.MECANUM) {
            // Mecanum Math
            double fl = drive + turn + strafe;
            double bl = drive + turn - strafe;
            double fr = drive - turn - strafe;
            double br = drive - turn + strafe;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), 
                         Math.max(Math.abs(fr), Math.abs(br))));
            if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }

            if (leftFront != null)  leftFront.setPower(fl);
            if (leftBack != null)   leftBack.setPower(bl);
            if (rightFront != null) rightFront.setPower(fr);
            if (rightBack != null)  rightBack.setPower(br);
        } else {
            // Tank Math
            double left = drive + turn;
            double right = drive - turn;
            setTankPowers(left, right);
        }
    }

    public void setTankPowers(double left, double right) {
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) { left /= max; right /= max; }
        
        for (DcMotorEx m : leftMotors) m.setPower(left);
        for (DcMotorEx m : rightMotors) m.setPower(right);
    }

    public void stop() {
        for (DcMotorEx m : allMotors) m.setPower(0.0);
    }

    public void driveInches(double inches, double power) {
        int target = (int) (inches * RobotConstants.COUNTS_PER_INCH);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (DcMotorEx m : allMotors) {
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setTankPowers(Math.abs(power), Math.abs(power));
    }

    public void turnDegreesApprox(double degrees, double power) {
        double inches = (degrees / 360.0) * (Math.PI * 12.0); // Assume 12" wheelbase
        int target = (int) (inches * RobotConstants.COUNTS_PER_INCH);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        for (DcMotorEx m : leftMotors) m.setTargetPosition(target);
        for (DcMotorEx m : rightMotors) m.setTargetPosition(-target);
        
        for (DcMotorEx m : allMotors) m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTankPowers(Math.abs(power), Math.abs(power));
    }

    public boolean isBusy() {
        for (DcMotorEx m : allMotors) {
            if (m.isBusy()) return true;
        }
        return false;
    }

    public void finishMotion() {
        stop();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotorEx m : allMotors) m.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx m : allMotors) m.setZeroPowerBehavior(behavior);
    }

    public int getLeftEncoder() {
        return leftMotors.isEmpty() ? 0 : leftMotors.get(0).getCurrentPosition();
    }

    public int getRightEncoder() {
        return rightMotors.isEmpty() ? 0 : rightMotors.get(0).getCurrentPosition();
    }
}
