package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@TeleOp(name = "Teste ArmSystem", group = "Test")
@Disabled
public class TesteArmSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosBot robot = new DestemidosBot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            robot.armSystem.moveArmsManually(gamepad1.right_stick_y);

            //telemetry.addData("arm - position", robot.armSystem.armA.getCurrentPosition());
            //telemetry.update();
        }
    }
}
