package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Autonomous(name="Auto", group="Linear OpMode")
public class autonanouse extends LinearOpMode {
private MecanumDrive mecanumDrive = null;
private DcMotor Launcher = null;
private DcMotor Intake = null;
private Servo Lift = null;
private CRServo Loader = null;

private GoBildaPinpointDriver pinPoint = null;
public void runOpMode() {
    Launcher = hardwareMap.get(DcMotor.class, "launcher");
    Intake = hardwareMap.get(DcMotor.class, "intake");
    Lift = hardwareMap.get(Servo.class, "lift");
    Loader = hardwareMap.get(CRServo.class, "loader");
    pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint");
//sets the start cords and heading of the robot
    Pose2d beginPose = new Pose2d(-48, 48, 225);
    mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

    waitForStart();
    Actions.runBlocking(
            mecanumDrive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(0, 0))
                    //TO DO SHOOT
                    .strafeToSplineHeading(new Vector2d(0,0), -90)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build());
}
       }
