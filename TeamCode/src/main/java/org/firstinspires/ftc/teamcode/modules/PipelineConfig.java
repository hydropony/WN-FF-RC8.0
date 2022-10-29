package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class PipelineConfig {
    //hsl
    public static int lhl = 0; //hls 0
    public static int lsl = 170; //hls 200
    public static int lvl = 0; //hls 0

    public static int hhl = 255; //hls 255
    public static int hsl = 255; //hls 255
    public static int hvl = 85; //hls 55

    //hsv
    public static int lhd = 20;
    public static int lsd = 100; //120
    public static int lvd = 100;

    public static int hhd = 40;
    public static int hsd = 255;
    public static int hvd = 255;


    //red
    public static int redlh = 0;
    public static int redls = 70;
    public static int redlv = 50;
    public static int redhh = 10;
    public static int redhs = 255;
    public static int redhv = 255;

    //blue
    public static int bluelh = 78;
    public static int bluels = 70;
    public static int bluelv = 50;
    public static int bluehh = 138;
    public static int bluehs = 255;
    public static int bluehv = 255;

    public static double MIN_WIDTH = 12; //(30.0 / 320.0) * CAMERA_WIDTH;

    /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
    public static double BOUND_RATIO = 0.7;

    public static double maxHeight = 50;



//    public static int lh = 0; //hls 0 hsv white
//    public static int ls = 0; //hls 200
//    public static int lv = 200; //hls 0
//
//    public static int hh = 255; //hls 255
//    public static int hs = 55; //hls 255
//    public static int hv = 255; //hls 55


//    public static Scalar lowerBound = new Scalar(lh, ls, lv);
//    public static Scalar upperBound = new Scalar(hh, hs, hv);
}
