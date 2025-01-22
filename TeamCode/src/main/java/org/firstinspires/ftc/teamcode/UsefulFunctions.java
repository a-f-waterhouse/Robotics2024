package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class UsefulFunctions extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        waitForStart();
        while (opModeIsActive() ) {

            //telemetry.addLine(String.format("position: %03d ", m.getCurrentPosition()));
            telemetry.update();
        }

    }



}
