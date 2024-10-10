## Welcome, I hope you enjoy ^^

## SAMPLES!!
you can copy/paste the files from:
FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples
to the folder that this is in.

The readme in there explains how they are organised :)

## Editing samples or writing your own
The structure of an OpMode is as such:

* imports
* autonomous vs teleop vs disabled
* class (name of opmode)
    * initialise objects (motors + such)
    * overriden runOpMode
      * hardware map
        * while opMode is active
          * write your code here!

## Imports
This is probably terrible advice, but even if your not starting with a sample, I'd copy/paste a samples
imports first and then just take out any that aren't used, to save you time and effort, and to save me from learning which ones are actually necessary!

## Type of OpMode
There may or may not be more, but the 3 important lines are:

``@TeleOp`` -> this declares it as an opmode to be used with a controller
``@Autonomous`` -> declares it asn autonomous opmode (wowwwww :0 /sarc)
``@Disabled``-> means it doesn't show up under the list of opmodes on the robot, so can't be run

## Declaring the main class

``public class <insert name here> extends LinearOpMode 
{
<code goes here!>
}``

## Initialising objects
Some of the main types we need are motors and servos, but later on we'll have to declare sensors + stuff too :D

Example:
``private Servo test_servo  = null;``
It's mostly self explanatory - you can also just miss out the ``=null`` if you want ¯\_(ツ)_/¯

## Override main program part
There are 2 parts to this.

1) say that you are overriding something with ``@Override``
2) declare subprogram: ``public void runOpMode() { <insert code here> }`` 

## Hardware Map
Inside the subprogram you have just declared, you have to map your hardware accordingly

Example:
``test_servo = hardwareMap.get(Servo.class, "test_servo");``

So essentially ``<name of object (as declared above)> = hardwareMap.get(<type of object>.class, "<name on driver hub config>");``
The important part is that the second parameter must match what you have called the thingimabob in the config settings on the Driver Hub

## (infinite?) Loop
The main place you need to put your code is with a ``while (opModeIsActive()){} loop``

## You're all set up!!
Congratulations, now you just need to write some code which actually *does* something!! (*^▽^*)