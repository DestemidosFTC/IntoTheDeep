package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSystem;

/**
 * Teste focado em experimentar o novo sistema de controle
 */
@TeleOp(name = "Teste - Field Oriented", group = "Test")
public class TesteFieldOriented extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        final Drivetrain drivetrain = new Drivetrain(hardwareMap);
        final LocalizationSystem localizationSystem = new LocalizationSystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {

            drivetrain.fieldOrientedController(gamepad1, localizationSystem.getRobotHeading());
            telemetry.update();
        }
    }
}
