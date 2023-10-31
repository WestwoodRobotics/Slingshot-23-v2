package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BasicDrive extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private Telemetry telemetry;
    IMU imu;
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



    }
    public void loop(){
        //get the input of the gamepad1 in order to move the robot
        double inputY = -gamepad1.left_stick_y; // Y stick is reversed
        double inputX = gamepad1.left_stick_x;
        double inputRot = gamepad1.right_stick_x;

        telemetry.addData("x y rot: ", inputX+", "+inputY+", "+inputRot);
        leftFront.setVelocity(inputY + inputX + inputRot);
        rightFront.setVelocity(inputY + inputX + inputRot);
        leftBack.setVelocity(inputY + inputX + inputRot);
        rightBack.setVelocity(inputY + inputX + inputRot);
    }

}
