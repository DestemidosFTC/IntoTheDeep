package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;


@TeleOp(name="7", group="Linear Opmode")

public class SemiGarra extends LinearOpMode {
    private GamepadEx player2;

    private ElapsedTime runtime = new ElapsedTime();

    private CRServoImplEx servo_E;
    private CRServoImplEx servo_D;

    double leftPower;
    double rightPower;





    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        player2 = new GamepadEx(gamepad2);

        servo_E = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_e");
        servo_D = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_d");

        servo_E.setDirection(CRServo.Direction.REVERSE);
        servo_D.setDirection(CRServo.Direction.FORWARD);

        servo_E.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        servo_D.setPwmRange(RobotConstants.MAX_SERVO_RANGE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            leftPower    = Range.clip(drive, 0, 0.5) ;

            rightPower   = Range.clip(drive, -1, 0) ;


                servo_E.setPower(leftPower);
                servo_D.setPower(rightPower);



        }
    }
}
