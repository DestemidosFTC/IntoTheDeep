package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "RotaL")
public class RotaLonga extends OpMode {
    private AprilTagDetection tagOfInterest = null;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;


    boolean aliancaVermelha = false;
    int multiplicadorVermelho = 1;
    private DestemidosBot robot;

    boolean tagFound = false;

    // configurando o hardware
    private AutonomoSystem driveAuto;

    TrajectorySequence ajuste;
    TrajectorySequence esque;
    TrajectorySequence verifDir;






    @Override
    public void init() {

        // habilita a telemetria no ftc-dashboard
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        // inicializa os sistemas
        robot = new DestemidosBot(hardwareMap);
        driveAuto = new AutonomoSystem(
                robot.drivetrain,
                robot.localizationSystem,
                robot.voltageSensor);

        //Invertendo os valores para a rota da Aliança vermelha no lado
        //esquerdo (visão de jogadores da aliança vermelha)

        if (aliancaVermelha){
            multiplicadorVermelho = -1;
        }

        // reseta o agendador de comandos
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(robot.armSystem, robot.servo);

        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

         ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d())
                 .forward(32)
                 .build();

         esque = driveAuto.trajectorySequenceBuilder( ajuste.end())
                .back(28)
                .turn(Math.toRadians(-220 * multiplicadorVermelho))
                .back(110)
                .strafeRight(30 * multiplicadorVermelho)
                .build();

         verifDir = driveAuto.trajectorySequenceBuilder( esque.end())
                .back(12)
                .turn(Math.toRadians(-60 * multiplicadorVermelho))
                .waitSeconds(2)
                .turn(Math.toRadians(60 * multiplicadorVermelho))
                .back(21)
                .turn(Math.toRadians(-225 * multiplicadorVermelho))
                .back(120)
                .strafeRight(30 * multiplicadorVermelho)
                .build();



        // configurando camera
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        // configurando a pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(
                RobotConstants.OPENCV_tagsize,
                RobotConstants.OPENCV_fx,
                RobotConstants.OPENCV_fy,
                RobotConstants.OPENCV_cx,
                RobotConstants.OPENCV_cy);

        // começa a stream da câmera pro ftc-dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        // define o pipeline e inicia a câmera
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(
                        RobotConstants.resolutionWidth,
                        RobotConstants.resolutionHeight,
                        OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
         */

        // definindo a posição inicial como (0,0,0) pro roadrunner
        driveAuto.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
    }

    @Override
    public void start() {

        driveAuto.followTrajectorySequence(ajuste);
        //while (true)
        // if (sensor color)
        //if (sensor color)

        driveAuto.followTrajectorySequence(verifDir);

    }

    @Override
    public void loop() {

    }

}
