package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    private Gamepad gamepad1;
    private IMU imu;
    private Telemetry telemetry;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    double heading;


    //auton only
    double idealHeading;
    boolean locked_heading = false;

    public Drive(Gamepad gamepad1, IMU imu, Telemetry telemetry, DcMotorEx LF, DcMotorEx LB, DcMotorEx RF, DcMotorEx RB)
    {
        this.gamepad1 = gamepad1;
        this.imu = imu;
        this.telemetry = telemetry;
        this.leftBack = LB;
        this.leftFront = LF;
        this.rightBack = RB;
        this.rightFront = RF;

    }
    public Drive(Gamepad gamepad1, IMU imu, Telemetry telemetry, DcMotorEx LF, DcMotorEx LB, DcMotorEx RF, DcMotorEx RB, boolean auton)
    {
        this.gamepad1 = gamepad1;
        this.imu = imu;
        this.telemetry = telemetry;
        this.leftBack = LB;
        this.leftFront = LF;
        this.rightBack = RB;
        this.rightFront = RF;
        locked_heading = auton;

    }

    public void update() {

        //get the input of the gamepad1 in order to move the robot
        double yInput = -gamepad1.left_stick_y; // Y stick is reversed
        double xInput = gamepad1.left_stick_x;
        double rotInput = gamepad1.right_stick_x;
        double rotOut = rotInput;
        //resets the imu if you press options
        if (gamepad1.options) {
            imu.resetYaw();
        }



        //sets the outputs to be field centric
        heading = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double xOut = xInput * Math.cos(-heading) - yInput * Math.sin(-heading);
        double yOut = yInput * Math.cos(-heading) + xInput * Math.sin(-heading);

        //outputs the x, y, and rotation and the heading of the robot (after field centric)
        telemetry.addData("x y rot: ", xOut + ", " + yOut + ", " + rotOut);
        telemetry.addData("heading: ", heading);

        //slowmode

        double slow = (gamepad1.left_trigger > 0.9) ? 1: 0.5;

        //sets the power of the motors
        leftFront.setPower((yOut + xOut + rotOut)*slow);
        rightFront.setPower((yOut - xOut + rotOut)*slow);
        leftBack.setPower((yOut - xOut - rotOut)*slow);
        rightBack.setPower((yOut + xOut - rotOut)*slow);
    }

    private double normalizeAngle(double angle) {
        // Normalizes angle in [0,360] range
        while (angle > 2*Math.PI) angle-=2*Math.PI;
        while (angle < 0) angle+= 2*Math.PI;
        return angle;
    }

    private double angleWrap(double angle) {
        // Changes any angle between [-180,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI)
            angle -= 2*Math.PI * (angle>0?1:-1);
        return angle;
    }

    private double calcRotBasedOnIdeal(double r) {

    }
}
