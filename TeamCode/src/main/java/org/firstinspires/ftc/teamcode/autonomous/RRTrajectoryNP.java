package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.detection.RightTseDetection;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(group = "No Pixels", name = "RRTrajectoryNP")
public class RRTrajectoryNP extends LinearOpMode {

    private DestemidosBot robot;
    private RightTseDetection rightTseDetection;
    public String line = "";

    TrajectorySequence MidSpikeMark;
    TrajectorySequence RightSpikeMark;
    TrajectorySequence LeftSpikeMark;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new DestemidosBot(hardwareMap);
        rightTseDetection = new RightTseDetection();

        Pose2d startPose = new Pose2d(21, -63, Math.toRadians(270));
        drive.setPoseEstimate(startPose);


        //TRAJETORIAS
        MidSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(16, -30, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(16, -35, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(57, -36, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(40); })
                .UNSTABLE_addTemporalMarkerOffset(-1.5,() -> { robot.servoSystem.fistServoRotation(165); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, -58, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(65, -58, Math.toRadians(180)), Math.toRadians(-180) )
                .build();


        RightSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(45, -30, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(35, -29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(59, -38, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(55); })
                .UNSTABLE_addTemporalMarkerOffset(-1.7,() -> { robot.servoSystem.fistServoRotation(150); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, -57, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(65, -57, Math.toRadians(180)), Math.toRadians(-180) )
                .build();


        LeftSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(20, -32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(14, -35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58, -26, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.7,() -> { robot.simpleArm.goToPosition(40); })
                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.servoSystem.fistServoRotation(150); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, -57, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(65, -57, Math.toRadians(180)), Math.toRadians(-180) )

                .build();



        //TENSORFLOW

        if (opModeInInit()) {
            rightTseDetection.initTfod(hardwareMap);
            while (opModeInInit()) {
                line = rightTseDetection.position( rightTseDetection.tfod);
                telemetry.addData(line, "Its here");

                if (line != null) {
                }
                telemetry.update();
                sleep(20);
            }
        }




        waitForStart();



        switch (line) {
            case "Center":
                drive.followTrajectorySequence(MidSpikeMark);
                break;
            case "Right":
                drive.followTrajectorySequence(RightSpikeMark);
                break;
            case "Left":
                drive.followTrajectorySequence(LeftSpikeMark);
                break;
        }


        terminateOpModeNow();


    }
}

