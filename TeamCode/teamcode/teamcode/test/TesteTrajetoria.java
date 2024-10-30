package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

/**
 * Teste focado em montar as trajetórias customizdas no RoadRunner
 * e testá-las ao vivo por aqui.
 */
@Autonomous(name = "TesteTrajetoria", group = "Test")
@Disabled
public class TesteTrajetoria extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DestemidosBot robot = new DestemidosBot(hardwareMap);
        AutonomoSystem autoRobot = new AutonomoSystem(robot.drivetrain, robot.localizationSystem, robot.voltageSensor);

        TrajectorySequence ir_para_pilha = autoRobot.trajectorySequenceBuilder( new Pose2d(0.0, 0.0, 0.0))
                .forward(60)
                .turn(UnitConversion.degreesToRadians(90.0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // se direciona para a pilha de cones
        autoRobot.followTrajectorySequence(ir_para_pilha);
    }
}
