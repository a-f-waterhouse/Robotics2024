package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class DriveTrain_Controller extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;
    private Servo claw_servo;

    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "initialised");
        telemetry.update();
        waitForStart();

        Boolean servoPos = false;

        while(opModeIsActive())
        {
            claw_servo.setPosition((servoPos)?1:0);
            if(gamepad1.b)
            {
                servoPos = !servoPos;
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            right_motor.setPower(y-x);
            left_motor.setPower(y+x);

            telemetry.addData("Status", "running");
            telemetry.update();
        }
    }

}
