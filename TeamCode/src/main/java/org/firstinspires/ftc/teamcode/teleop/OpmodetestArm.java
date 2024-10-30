package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class OpmodetestArm extends CommandOpMode {
    private PIDController controller;

    public static double p = 0.1, i = 0, d = 0.0015;
    public static double f = 0.8;

    public static int target = 0;

    public final double ticks_in_degree = 360 / 288.0;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;




    @Override
    public void initialize() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, "fore");
        rightMotor = hardwareMap.get(DcMotorEx.class, "forearm");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void run() {
        controller.setPID(p, i, d);
        int leftArmPos = leftMotor.getCurrentPosition();
        int rightArmPos = rightMotor.getCurrentPosition();
        int armPos = (leftArmPos + rightArmPos) / 2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        telemetry.addData("position", armPos);
        telemetry.addData("target", target);
        telemetry.update();




    }


}