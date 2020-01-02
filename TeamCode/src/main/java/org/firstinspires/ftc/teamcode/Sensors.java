package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensors")
public class Sensors extends Autonomous2020 {

    public void distanceToUnknownObstacles(){
        double rfStraight = distanceSensor_rf_Straight.getDistance(DistanceUnit.INCH);
        double rbStraight = distanceSensor_rb_Straight.getDistance(DistanceUnit.INCH);
        double lfStraight = distanceSensor_lf_Straight.getDistance(DistanceUnit.INCH);
        double lbStraight = distanceSensor_lb_Straight.getDistance(DistanceUnit.INCH);

    }


}
