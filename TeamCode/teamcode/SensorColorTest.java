package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "TestColor2", group = "Group")


public class SensorColorTest extends LinearOpMode {
    public ColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.colorSensor.get("sensor_color");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("red: ", color.red());
            telemetry.addData("green: ", color.green());
            telemetry.addData("blue ", color.blue());
        }
    }


}
