package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
public class PinPoint_Execution {
    GoBildaPinpointDriver odo;
    double lastXError = 0;
    double lastYError = 0;

    // Constructor
    public PinPoint_Execution(GoBildaPinpointDriver odo) {
        this.odo = odo;
        odo.setOffsets(-100, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }


    public List<Double> move(double x, double y, double z) {
        odo.setOffsets(-100, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        Pose2D pos = odo.getPosition();
        double xError = pos.getX(DistanceUnit.MM) - x;
        double yError = pos.getY(DistanceUnit.MM) - y;
        double zError = pos.getHeading(AngleUnit.DEGREES) - z;
        double xP = -0.25 * xError;
        double yP = -0.25 * yError;
        double xD = -0.25 * (xError - lastXError) / odo.getLoopTime();
        double yD = -0.25 * (yError - lastYError) / odo.getLoopTime();

        lastXError = xError;
        lastYError = yError;

        double linXVel = xP + xD;
        double linYVel = yP + yD;
        double fLVelocity = linXVel - linYVel;
        double fRVelocity = linXVel + linYVel;
        double rLVelocity = linXVel + linYVel;
        double rRVelocity = linXVel - linYVel;
        List<Double> velocity = Arrays.asList(fLVelocity, fRVelocity, rLVelocity, rRVelocity);
        return velocity;
    }
}
