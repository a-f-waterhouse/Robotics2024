package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;



@Autonomous
public class Main_Autonomous extends LinearOpMode {

    public class Robot
    {
        public int heading;
        public int[] position;

        public String currentState;
    }

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
    
    int[] armRotatePos = {}; //up, down

    private int CountsPerCM = 30; //NEED TO CALCULATE

    //objects
    private Robot bpv;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor left_drive;
    private DcMotor right_drive;    
    private DcMotor arm_rotate;    
    private DcMotor arm_extension;    
    private Servo claw_servo;
    private IMU imu;

    @Override
    public void runOpMode()
    {
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
            initAprilTag();
            telemetry.addData("Status", "initialised");
            telemetry.update();
        }

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("State",bpv.currentState);
            switch(bpv.currentState)
            {
                
            }
            telemetry.update();
        }
        visionPortal.close();
    }
    public void MoveToCoords(double targetHeading, int[] coords)
    {
        int distance = (int)(Math.sqrt(Math.pow(bpv.position[0],2) + Math.pow(bpv.position[1],2)));
        Turn(targetHeading);
        MoveForward(distance,0,1);
    }

    public void MoveForward(double distance, double heading, double maxSpeed)
    {
        if(opModeIsActive())
        {
            int moveCounts = (int)(distance * CountsPerCM);
            int leftTarget = left_drive.getCurrentPosition() + moveCounts;
            int rightTarget = right_drive.getCurrentPosition() + moveCounts; //new target positions

            left_drive.setTargetPosition(leftTarget);
            right_drive.setTargetPosition(rightTarget);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Move(maxSpeed, 0);
        }
        Move (0,0);

    }

    public void Turn(double heading)
    {
        if(opModeIsActive()) {
            double currentBearing = imu.getRobotYawPitchRollAngles().getYaw();
            while (currentBearing < heading)
            {
                Move(0,0.3);
            }
        }

    }
    public void Move(double drive, double turn)
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

    }
    public void GrabWithClaw()
    {


    }

    public void MoveArm(boolean down)
    {
        arm_rotate.setTargetPosition(armRotatePos[(down ? 1 : 0)]);
        arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_rotate.setPower(0.5);
    }

    public void MoveClaw(boolean open)
    {
        claw_servo.setPosition(DegreesToPos(servoPos[(open ? 1 : 0)]));
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public String DetermineIntitialPosition()
    {
        int[] red = {};
        int[] blue = {};
        String pos = "";

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection d:currentDetections)
        {
            //if(Arrays.stream(red).anyMatch())
            {

            }
        }


        return "";
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



}
