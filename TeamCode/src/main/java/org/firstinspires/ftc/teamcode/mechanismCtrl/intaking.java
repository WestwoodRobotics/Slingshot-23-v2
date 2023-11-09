package org.firstinspires.ftc.teamcode.mechanismCtrl;

import org.firstinspires.ftc.teamcode.constants.slideConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;




public class intaking {
    DcMotorEx slideMotor;

    //needs tuning
    final double kP = 5.0;
    final double kI = 0.0;
    final double kD = 0.0;
    final double kF = 0.0;

        public intaking(DcMotorEx slideMotor) {
            this.slideMotor = slideMotor;
            slideMotor.setTargetPosition(-31);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kP, kI, kD, kF));
        }

}
