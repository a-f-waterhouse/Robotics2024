package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Controller_Test extends LinearOpMode {

    private DcMotor test_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        test_motor = hardwareMap.get(DcMotor.class, "test_motor");

        telemetry.addData("Status", "initialised");
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {
            double power  = -gamepad1.right_stick_y ;
            test_motor.setPower(power);
            telemetry.addData("Motor position", test_motor.getCurrentPosition());
            if(gamepad1.y)
            {
                test_motor.setPower(0.5);
                wait(1000); //??
            }
        }
    }

}
