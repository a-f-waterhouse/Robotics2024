package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class Main_Autonomous extends LinearOpMode {
    private int[][] AprilTagCoords =
    {
        {-72, 48},
        {0, 72},
        {72, 48},
        {72, -48},
        {0, -72},
        {-72, -48}

    };
    int[]  servoPos = {-30,30}; //open, close
    int[] armPos = {480,5}; //out, in
    int[] armRotatePos = {-1350, -5}; //up, down
    private static final int CountsPerInch = 1; //NEED TO CALCULATE
    private static final int CountsPerDegree = 1; //NEED To CALCULATE
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor left_drive;
    private DcMotor right_drive;    
    private DcMotor arm_rotate;    
    private DcMotor arm_extension;    
    private Servo claw_servo;
    private IMU imu;
    private ElapsedTime timer = new ElapsedTime();
    String State = "";

    @Override
    public void runOpMode() throws InterruptedException {

        left_drive = hardwareMap.get(DcMotor.class, "left");
        right_drive = hardwareMap.get(DcMotor.class, "right");
        claw_servo = hardwareMap.get(Servo.class, "claw");
        arm_extension = hardwareMap.get(DcMotor.class, "arm");
        arm_rotate = hardwareMap.get(DcMotor.class, "armRotate");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        initAprilTag();

        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            String initPos = DetermineInitialPosition();
            telemetry.addData("Status", "initialised");
            telemetry.addData("Start position", initPos);
            telemetry.update();
            switch(initPos)
            {
                case "Clip":
                    State = "ClipToSub";
                    break;
                case "Score":
                    State = "ScoreBaskets";
                    break;
                default:
                    State = "";

            }
        }

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        waitForStart();

        while(opModeIsActive()) //TURN = positive = left
        {
            telemetry.addData("State", State);
            telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
            telemetry.update();
            switch (State)
            {
                case "ClipToSub":
                    ClippingToSub();
                    State = "ObservationZone";
                    break;
                case "ScoreBaskets":
                    ScoringBaskets();
                    State = "AscentZone";
                    break;
                case "ObservationZone":
                    ReturnToOZ();
                    State = "";
                    break;
                case "AscentZone":
                    GoToAZ();
                    break;
            }
            requestOpModeStop();

        }
        visionPortal.close();
    }

    public void ClippingToSub()
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(75,0.05); //80= 90
            MoveForward(1000,0.3);
            TurnWithEncoders(-30,0.05); //80= 90
            Move(0,0);
            RotateArm(false);
            CheckBusy();
            ExtendArm(false);
            CheckBusy();
            /*RotateArm(true); //CHECK ANGLES!!
            CheckBusy();*/
            MoveClaw(true);
            ExtendArm(true);;
            RotateArm(true);;
        }
    }

    public void ReturnToOZ() //observation zone
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(-135, 0.05);
            MoveForward(100, 0.3);
            TurnWithEncoders(180, 0.05);
        }
    }

    public void ScoringBaskets()
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(10, 0.05);
            MoveForward(500, 0.3);
            RotateArm(false);
            ExtendArm(false);
            MoveForward(20, 0.1);
            MoveClaw(true);
            ExtendArm(true);
            RotateArm(true);

        }
    }

    public void GoToAZ() //Ascent Zone
    {
        if(opModeIsActive())
        {
            TurnWithEncoders(-100, 0.05); //100 degrees right
            MoveForward(500, 0.3); //2m
            TurnWithEncoders(-80, 0.05); //80 degrees right
            MoveForward(100, 0.05); //0.5m
            TurnWithEncoders(180, 0.05); //180 degrees
        }
    }




    public void MoveForward(double distance, double maxSpeed) //distance in inches
    {
        if(opModeIsActive()) {
            int moveCounts = (int) (distance * CountsPerInch);
            int leftTarget = left_drive.getCurrentPosition() + moveCounts;
            int rightTarget = right_drive.getCurrentPosition() + moveCounts; //new target positions

            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Move(maxSpeed, 0);
        }

    }


    public void TurnWithEncoders(double heading, double speed) //heading between -180 nad 180 relative to robot
    {
        if(opModeIsActive())
        {
            int leftTarget, rightTarget;
            if(heading < 0) //turn right
            {
                leftTarget = (int)(CountsPerDegree * heading) + left_drive.getCurrentPosition();
                rightTarget = right_drive.getCurrentPosition()-(int)(CountsPerDegree * heading);
            }
            else //left
            {
                rightTarget = (int)(CountsPerDegree * heading) + right_drive.getCurrentPosition();
                leftTarget = left_drive.getCurrentPosition()-(int)(CountsPerDegree * heading);
            }
            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_drive.setPower(speed);
            right_drive.setPower(speed);
            CheckBusy();
        }

    }

    public void RotateArm(boolean down)
    {
        if(opModeIsActive())
        {
            arm_rotate.setTargetPosition(armRotatePos[(down ? 1 : 0)]);
            arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(down)
            {
                arm_rotate.setPower(0.5);
            }
            else
            {
                arm_rotate.setPower(-0.5);
            }
            CheckBusy();
        }
    }

    public void ExtendArm(boolean in)
    {
        if(opModeIsActive())
        {
            arm_extension.setTargetPosition(armPos[(in ? 1 : 0)]);
            arm_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(in)
            {
                arm_extension.setPower(-0.5);
            }
            else
            {
                arm_extension.setPower(0.5);
            }
            CheckBusy();
        }
    }

    public void MoveClaw(boolean open)
    {
        claw_servo.setPosition(DegreesToPos(servoPos[(open ? 1 : 0)]));
    }
    public void Move(double drive, double turn)
    {
        if(opModeIsActive())
        {
            double leftSpeed = drive + turn;
            double rightSpeed = drive - turn;

            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            left_drive.setPower(leftSpeed);
            right_drive.setPower(rightSpeed);
            CheckBusy();
        }

    }

    public double GetHeading(){ //-180 to 180
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void CheckBusy()
    {
        if(opModeIsActive())
        {
            while(left_drive.isBusy() || right_drive.isBusy())
            {
                telemetry.addData("State", State);
                telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
                telemetry.update();
            }
            while(arm_rotate.isBusy() || arm_extension.isBusy())
            {
                telemetry.addData("State", State);
                telemetry.addData("Heading: ", "%7f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("left, right", "%7d :%7d", (left_drive.getCurrentPosition()), (right_drive.getCurrentPosition()));
                telemetry.update();
            }
            Move(0,0);
            timer.reset();
            while(timer.seconds() < 1){}
        }

    }

    public String DetermineInitialPosition()
    {
        int[] Clip = {11,14};
        int[] Score = {13,16};

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection d:currentDetections)
        {
            if(Arrays.asList(Clip).contains(d.id))
            {
                return "Clip";
            }
            else if(Arrays.asList(Score).contains(d.id))
            {
                return "Score";
            }
            else
            {
                return "Unknown";
            }
        }
        return "NoDetections";
    }


    private void initAprilTag() //inches + radians
    {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


    }


    private double DegreesToPos(double degrees) //0 = -135, 1 = 135
    {
        return (degrees + 135)/270;
    }

    public double NormaliseAngle(double angle) //returns angle between 180 and -180
    {
        if(angle > 180)
        {
            angle -= 360;
        }
        else if(angle < -180)
        {
            angle += 360;
        }
        return angle;

    }




}
