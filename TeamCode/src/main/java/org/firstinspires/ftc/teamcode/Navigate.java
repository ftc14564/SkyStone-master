package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class Navigate {

    Navigate(Autonomous2020 auto){
        autoBase = auto;
    }

    Autonomous2020 autoBase;
    int robotx = 18;
    int roboty = 18;
    int robotz = 18;
    boolean [][] field = new boolean [144][144];
    public void addingKnownObstacles(){
        for(int h = 0; h < 2; h++) {
            int multiplier = h *24;
            for (int i = 57; i <= 63; i++) {
                for (int j = 45; j <= 51; j++) {

                    field[i + multiplier][j] = true;
                    field[i + multiplier][j] = true;
                }
            }
        }
        //adding bridge poles

        for(int h = 0; h < 2; h++) {
            int multiplier = h *24;
            for (int i = 57; i <= 63; i++) {
                for (int j = 93; j <= 99; j++) {

                    field[i + multiplier][j] = true;
                    field[i + multiplier][j] = true;
                }
            }
        }
        //adding bridge poles

        for(int i = 0;i<144;i++){
            field[0][i] = true;
        }
        //adding Wall

        for(int i = 0;i<144;i++){
            field[i][0] = true;
        }
        //Adding Wall

    }
    public boolean goTo(double x, double y){
        
//        autoBase.EncoderStraight(Math.abs(autoBase.where_y-y));
//        if(autoBase.where_x < x){
//            autoBase.Rotate(1,1,90);
//        }
//        else{
//            autoBase.Rotate(1,-1,90);
//        }
//        autoBase.EncoderStraight(Math.abs(autoBase.where_x-x));

        return false;
    }


    public void abort(){

    }

    public double currentX(){
        return autoBase.where_x;
    }

    public double currentY(){
        return autoBase.where_y;
    }


}
