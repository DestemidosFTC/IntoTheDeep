package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.detection.LeftTseDetection;
import org.firstinspires.ftc.teamcode.commands.simplecommands.ArmCommand;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(group = "No Pixels", name = "BLTrajectoryNP")
public class BLTrajectoryNP extends LinearOpMode {

    private DestemidosBot robot;
    private LeftTseDetection leftTseDetection;
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
        leftTseDetection = new LeftTseDetection();

        Pose2d startPose = new Pose2d(21, 63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        CommandScheduler.getInstance().reset();
        //TRAJETORIAS
        MidSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(16, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(16, 35, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(59.5, 41, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(40); })
                .UNSTABLE_addTemporalMarkerOffset(-1.5,() -> { robot.servoSystem.fistServoRotation(165); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() -> { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, 60, Math.toRadians(180)))
                .setReversed( true )
                .splineToLinearHeading(new Pose2d(64, 63, Math.toRadians(180)), Math.toRadians(-180) )
                .build();


        RightSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(20, 32, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12, 35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58, 31, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.7,() -> { robot.simpleArm.goToPosition(55); })
                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.servoSystem.fistServoRotation(140); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, 60, Math.toRadians(180)))
                .setReversed( true )
                .splineToLinearHeading(new Pose2d(61, 62, Math.toRadians(180)), Math.toRadians(-180))
                .build();



        LeftSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.2,() -> { robot.servoSystem.closeWrist(); })
                //o comando acima é apenas para fechar o intake no início da trajetória, garantindo que o pixel não caia

                .lineToLinearHeading(new Pose2d(45, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(32, 30, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58.5, 44, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1.8,() -> { robot.simpleArm.goToPosition(45); })
                .UNSTABLE_addTemporalMarkerOffset(-1.7,() -> { robot.servoSystem.fistServoRotation(150); })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> { robot.servoSystem.openWrist(); })

                .UNSTABLE_addTemporalMarkerOffset(0.5,() ->  { robot.servoSystem.closeWrist(); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.servoSystem.fistServoRotation(-200); })
                .UNSTABLE_addTemporalMarkerOffset(1.0,() ->  { robot.simpleArm.goToPosition(0); })

                .back(10)
                .lineToLinearHeading(new Pose2d(40, 60, Math.toRadians(180)))
                .setReversed( true )
                .splineToLinearHeading(new Pose2d(61, 62, Math.toRadians(180)), Math.toRadians(-180) )
                .build();




        //TENSORFLOW

        if (opModeInInit()) {
            leftTseDetection.initTfod(hardwareMap);
            while (opModeInInit()) {
                line = leftTseDetection.position( leftTseDetection.tfod);
                telemetry.addData(line, "Its here");

                if (line != null) {
                }
                telemetry.update();
                sleep(20);
            }
        }




        waitForStart();
        CommandScheduler.getInstance().run();


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

