package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class UsefulFunctions extends LinearOpMode {

    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    private Servo test_servo  = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        test_servo = hardwareMap.get(Servo.class, "test_servo");

        waitForStart();
        while (opModeIsActive()) {
            test_servo.setPosition(DegreesToPos(45));
        }

    }

}
