package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="Basic: Linear OpMode", group="Linear OpMode")
public abstract class autonanouse extends LinearOpMode {
private DcMotor leftRearDrive = null;
private DcMotor rightRearDrive = null;
private DcMotor leftFrontDrive = null;
private DcMotor rightFrontDrive = null;
private DcMotor Launcher = null;
private DcMotor Intake = null;
private Servo Lift = null;
public void runOpMode(){
    leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
    rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
    Launcher = hardwareMap.get(DcMotor.class, "launcher");
    Intake = hardwareMap.get(DcMotor.class, "intake");
    Lift = hardwareMap.get(Servo.class, "lift");

    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
 }
}
