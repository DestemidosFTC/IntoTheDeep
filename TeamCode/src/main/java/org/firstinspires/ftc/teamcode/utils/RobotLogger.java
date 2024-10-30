package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * RobotLogger - Responsável por transmitir informações dos
 * devidos dispositivos para o Telemetry, como forma de ajudar
 * a identificar possiveis problemas na hora de testatgem ou debug
 * dos OpModes.
 */
public final class RobotLogger {

    public static void showSimpleMotorInfo(Telemetry telemetry, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            telemetry.addData("Motor Info: ", "%s", motor.getDeviceName());
            telemetry.addData("Power: ", motor.getPower());
            telemetry.addData("Position: ", motor.getCurrentPosition());
        }
    }

    public static void showFulleMotorInfo(Telemetry telemetry, DcMotorEx... motors){
        for (DcMotorEx motor : motors) {
            showSimpleMotorInfo(telemetry, motor);
            telemetry.addData("Target Postion: ", motor.getTargetPosition());
            telemetry.addData("Current: ", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("Direction", motor.getDirection());
            telemetry.addData("Port", motor.getPortNumber());
        }
    }

    public static void debugServoExInfo(Telemetry telemetry, ServoEx servo) {
        telemetry.addData("Servo Position:", servo.getPosition());
        telemetry.addData("Servo Direction:", servo.getAngle());
        telemetry.addData("Is Inverted", servo.getInverted());
    }

    // Invertemos o valor do Y, por causa que o SDK define
    // para cima como "-1.0", e para baixo como "+1.0"
    public static void debugControles(Telemetry telemetry, Gamepad controller1, Gamepad controller2) {
        telemetry.addLine("\nGAMEPADS: ");
        telemetry.addData("Gamepad1:",
                "Y: %.2f  X: %.2f  Giro: %.2f",
                controller1.left_stick_y,
                controller1.left_stick_x,
                controller1.right_stick_x
        );

        telemetry.addData("Gamepad2:",
                "Y: %.2f  X: %.2f  Giro: %.2f",
                controller2.left_stick_y,
                controller2.left_stick_x,
                controller2.right_stick_x
        );
    }
}
