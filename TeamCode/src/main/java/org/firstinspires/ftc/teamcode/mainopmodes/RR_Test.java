package org.firstinspires.ftc.teamcode.mainopmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous
public class RR_Test extends LinearOpMode
{
    public class Arm
    {
        private final int[] Extensions = {0,0}; // out, in
        private final int[] Rotations = {0,0}; // up, down
        private DcMotorEx extendMotor;
        private  DcMotorEx rotateMotor;
        public Arm(HardwareMap hardwareMap)
        {
            extendMotor = hardwareMap.get(DcMotorEx.class, "arm");
            rotateMotor = hardwareMap.get(DcMotorEx.class, "rotate");

            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public class ArmUp implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(-0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos < Rotations[0])
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ArmDown implements Action
        {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!init)
                {
                    rotateMotor.setPower(0.5);
                    init = true;
                }
                double pos = rotateMotor.getCurrentPosition();
                packet.put("Rotation", pos);

                if(pos < Rotations[1])
                {
                    return true;
                }
                else
                {
                    rotateMotor.setPower(0);
                    return false;
                }
            }
        }

        public class ArmExtend implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){ return false;} //!! TODO add functionality extend
        }
        public class ArmRetract implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){ return false;} // TODO add functionality retract
        }

        public Action armUp()
        {
            return new ArmUp();
        }
        public Action armDown()
        {
            return new ArmDown();
        }
        public Action armExtend()
        {
            return new ArmExtend();
        }
        public Action armRetract()
        {
            return new ArmRetract();
        }
    }

    public class Claw
    {
        private Servo claw;
        private final int[] Positions = {0,0}; //open, closed

        public Claw(HardwareMap hardwareMap)
        {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class OpenClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                claw.setPosition(Positions[(0)]);
                return false;
            }
        }

        public class CloseClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                claw.setPosition(Positions[(1)]);
                return false;
            }
        }

        public Action open()
        {
            return new OpenClaw();
        }
        public Action close()
        {
            return new CloseClaw();
        }


    }


    @Override
    public void runOpMode()
    {
        Pose2d initialPos = new Pose2d(0, 0, Math.toRadians(90));
        TankDrive drive = new TankDrive(hardwareMap, initialPos);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        //init trajectory stuffs
        TrajectoryActionBuilder tabTest = drive.actionBuilder(initialPos)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        Actions.runBlocking(claw.close()); //happens on init
        while(opModeInInit())
        {
            telemetry.addData("Starting Position", initialPos);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        Action trajectory = tabTest.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectory,
                        arm.armExtend()
                        //add more actions here etc.
                )
        );


    }



}
