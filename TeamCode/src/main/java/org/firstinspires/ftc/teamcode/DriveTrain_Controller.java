package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class DriveTrain_Controller extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;

    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "initialised");
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;

            right_motor.setPower(y+x);
            left_motor.setPower(y-x);

            telemetry.addData("Status", "running");
            telemetry.update();
        }
    }

}
