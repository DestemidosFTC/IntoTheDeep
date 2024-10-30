package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.detection.LeftTseDetection;
import org.firstinspires.ftc.teamcode.autonomous.detection.RightTseDetection;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(group = "No Pixels", name = "BRTrajectoryNP")
public class BRTrajectoryNP extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-42, 63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        //TRAJETORIAS
        MidSpikeMark = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(-37, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-37, 36, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(49, 38, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(45); })
                .UNSTABLE_addTemporalMarkerOffset(-1.5,() -> { robot.servoSystem.fistServoRotation(160); })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.simpleArm.goToPosition(30); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, 59, Math.toRadians(180)))
                .setReversed( true )
                .splineToLinearHeading(new Pose2d(55, 59, Math.toRadians(180)), Math.toRadians(-180) )
                .build();


        RightSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(-35, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-42, 30, Math.toRadians(0)))
                .forward(5)
                .lineToLinearHeading(new Pose2d(-37, 8, Math.toRadians(0)))
                .forward(10)
                .splineTo(new Vector2d(52, 30), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(45); })
                .UNSTABLE_addTemporalMarkerOffset(-1.5,() -> { robot.servoSystem.fistServoRotation(140); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(30); })

                .lineToLinearHeading(new Pose2d(45, 64, Math.toRadians(180)))
                .build();



        LeftSpikeMark = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(-45, 35, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(180)))
                .forward(10)
                .splineTo(new Vector2d(-14, 8), Math.toRadians(0))
                .forward(28)
                .splineTo(new Vector2d(51.5, 33), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1.4,() -> { robot.simpleArm.goToPosition(45); })
                .UNSTABLE_addTemporalMarkerOffset(-1.4,() -> { robot.servoSystem.fistServoRotation(150); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(0); })

                .back(8)
                .lineToLinearHeading(new Pose2d(45, 52, Math.toRadians(180)))
                .setReversed( true )
                .splineToLinearHeading(new Pose2d(55, 52, Math.toRadians(180)), Math.toRadians(-180) )
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

