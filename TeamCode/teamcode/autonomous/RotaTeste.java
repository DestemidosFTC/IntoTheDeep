package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleArm;

@Autonomous(name = "Rota Teste")
public class RotaTeste extends LinearOpMode {
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DestemidosBot robot = new DestemidosBot(hardwareMap);
        SimpleArm simpleArm = new SimpleArm(hardwareMap);
        AutonomoSystem drive = new AutonomoSystem(robot.drivetrain, robot.localizationSystem, robot.voltageSensor);

        // a idéia é testar esse DisplacementMarker e se ele realmente tá executando a função
        final TrajectorySequence etapa_1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(45)
                .turn(-Math.toRadians(48))
                .addDisplacementMarker(() -> {
                    simpleArm.goToPosition(RobotConstants.ARM_HIGH_GOAL);
                })
                .waitSeconds(5)
                .build();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        if (isStopRequested()) return;

        // agora é só testar
        drive.followTrajectorySequence(etapa_1);
        terminateOpModeNow();
    }
}
