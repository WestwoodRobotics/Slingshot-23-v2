package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BasicDrive extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private Telemetry telemetry; //idk how telemetry works
    IMU imu;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    @Override
    public void init(){
        //initializes the motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        //reverses the left motors
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        //brakes when stopped
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //imu initialized
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_FACING_DIR, USB_FACING_DIR));
        imu.initialize(parameters);


    }
    public void loop(){
        //get the input of the gamepad1 in order to move the robot
        double y = -gamepad1.left_stick_y; // Y stick is reversed
        double x = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;


        //resets the imu if you press options
        if(gamepad1.options) imu.resetYaw();

        //sets the outputs to be field centric
        double heading = normalizedHeading();
        x = x * Math.cos(-heading) - y * Math.sin(-heading);
        y = y * Math.cos(-heading) + x * Math.sin(-heading);

        //outputs the x, y, and rotation and the heading of the robot (after field centric)
        telemetry.addData("x y rot: ", x+", "+y+", "+rot);
        telemetry.addData("heading: ", heading);
        //sets the power of the motors
        leftFront.setVelocity(y + x + rot);
        rightFront.setVelocity(y + x + rot);
        leftBack.setVelocity(y + x + rot);
        rightBack.setVelocity(y + x + rot);
    }

    private double normalizedHeading(){
        double fixedAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (fixedAngle > 2 * Math.PI) fixedAngle -= 2*Math.PI;
        while (fixedAngle < 0) fixedAngle += 2*Math.PI;
        return fixedAngle;
    }
}
