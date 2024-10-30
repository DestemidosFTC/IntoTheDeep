package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;


@Config
@TeleOp
public class OpmodetestFist extends CommandOpMode{

    public static double degrees;

    CRServoImplEx wristServoC;
    CRServoImplEx wristServoD;

    GamepadEx player2;

    ElapsedTime fistTimer;




    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        player2 = new GamepadEx(gamepad2);
        fistTimer = new ElapsedTime();

        wristServoC = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoD"); //Porta 3, Servo
        wristServoD = (CRServoImplEx) hardwareMap.get(CRServoImplEx.class, "servoE"); //Porta 4, Servo
    }


    @Override
    public void run() {

        telemetry.addData("degrees", degrees);
        telemetry.update();

        fistTimer.reset();

        double time = RobotConstants.degreesToSeconds(degrees);
        int power = 1;

        if(time < 0) {
            power = -1;
            time = time * -1;
        }

        while (fistTimer.seconds() < time) {
            wristServoC.setPower(power);
            wristServoD.setPower(power);
        }

        wristServoC.setPower(0);
        wristServoD.setPower(0);
        wristServoC.setPwmDisable();
        wristServoD.setPwmDisable();
        fistTimer.reset();


    }


}