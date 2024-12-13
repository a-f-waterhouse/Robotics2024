package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class DriveTrain_Controller extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;

    private DcMotor arm_motor;
    private Servo claw_servo;
    private ElapsedTime     bPressed = new ElapsedTime();

    private ElapsedTime     aPressed = new ElapsedTime();

    private int[] servoPositions = {0,45};

    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    private int MapJoystickToMotor(double input)
    {

        if(input > 0)
        {
            return (int)(input * 600);
        }
        else{
            return 0; //(int)((input +1) * 600);
        }

    }


    @Override
    public void runOpMode() {
        left_motor = hardwareMap.get(DcMotor.class, "left");
        right_motor = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");
        arm_motor = hardwareMap.get(DcMotor.class, "arm");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "initialised");
        telemetry.update();


        waitForStart();

        int[]  servoPos = {-30,30};
        int[] armPos = {1250,0};
        int s = 0;
        int a = 0;

        while(opModeIsActive())
        {
            if(gamepad1.b)
            {
                if(bPressed.seconds() >= 0.5)
                {
                    s++;
                    s%=2;
                    bPressed.reset();
                }
            }
            /*if(gamepad1.a)
            {
                if(aPressed.seconds() >= 0.5)
                {
                    a++;
                    a%=2;
                    arm_motor.setTargetPosition(armPos[a]);
                    arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm_motor.setPower(0.5);
                    aPressed.reset();
                }

            }*/
            if(gamepad1.right_stick_y > 0)
            {
                arm_motor.setTargetPosition(MapJoystickToMotor(gamepad1.right_stick_y));
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);
            }
            else if(gamepad1.right_stick_y < 0)
            {
                arm_motor.setTargetPosition(0);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)                                                                                                                                                                                                                                 ;
                arm_motor.setPower(0.5);
            }
            else if(!arm_motor.isBusy()) {
                arm_motor.setPower(0);
            }


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            right_motor.setPower(y-x);
            left_motor.setPower(y+x);
            claw_servo.setPosition(DegreesToPos(servoPos[s]));

            telemetry.addData("Status", "running");
            telemetry.update();
        }
    }

}
