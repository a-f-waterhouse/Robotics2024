package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class Tank_Drive extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;

    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");

        //right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "initialised");
        telemetry.update();
        waitForStart();

        while(opModeIsActive())
        {
            double right = gamepad1.right_stick_y;
            double left = -gamepad1.left_stick_y;


            right_motor.setPower(right);
            left_motor.setPower(left);


            telemetry.addData("Status", "running");
            telemetry.update();
        }
    }

}
