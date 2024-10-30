package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;


@Config
@TeleOp
public class OpmodetestIntake extends CommandOpMode {

    GamepadEx player2;

    ServoImplEx wristServoA;
    ServoImplEx wristServoB;

    public double position = wristServoA.getPosition();





    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        player2 = new GamepadEx(gamepad2);

        wristServoA = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_d"); //Porta 0, Servo
        wristServoB = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "servo_e");
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        player2.readButtons();

        double position = wristServoA.getPosition();

        telemetry.addData("position", position);
        telemetry.update();




    }


}