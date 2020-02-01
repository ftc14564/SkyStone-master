package org.firstinspires.ftc.teamcode;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import static org.opencv.imgcodecs.Imgcodecs.imread;


import org.opencv.core.Point;





import android.content.Context;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;
import android.graphics.Bitmap;
import android.graphics.Color;




import com.vuforia.Frame;
import com.vuforia.Image;


public class CVUtil {

    private boolean mIsColorSelected = false;
    private Mat mRgba;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;
    private ColorBlobDetector mDetector;
    private Mat mSpectrum;
    private Size SPECTRUM_SIZE;
    private Scalar CONTOUR_COLOR;

    private CameraBridgeViewBase mOpenCvCameraView;


    private static final String TAG = "CVUtil";


    private Mat m_imageMat;
    private Context m_context;
    private BaseLoaderCallback mLoaderCallback;


    public void initCv(Context context) {


        m_context = context;
        mLoaderCallback = new BaseLoaderCallback(context) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        Log.i("OpenCV", "OpenCV loaded successfully");
                        m_imageMat = new Mat();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }

        };

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, context, mLoaderCallback);
        }


    }


    public void onResume() {
        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, m_context, mLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }


    Date dtLastTimePic = new Date();

    Point updateFrame(Bitmap bm, Frame frame) {


        Image img = frame.getImage(0);
        Mat hsvMat = new Mat();
        Mat blurredMat = new Mat();
        Mat mask = new Mat();
        Mat mask2 = new Mat();
        Mat combine = new Mat();
        Mat morphOutput = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Point centerPoint = new Point(0, 0);



        Mat mat = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, mat);


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        System.out.println("LOOK HERE : BEFORE ERROR");
        try{
//            StoneWrangler stoneWrangler = new StoneWrangler();
//            stoneWrangler.analyze(mat);
//            Mat sMat = stoneWrangler.getVisualization();
        } catch (Exception e){
            System.out.println(e.toString() + "HEY LOOK HERE");

        }
        System.out.println("LOOK HERE : AFTER ERROR");


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
        Imgproc.blur(mat, blurredMat, new Size(7, 7));
        Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_BGR2HSV);

        Scalar minValues = new Scalar(0, 70, 50);
        Scalar maxValues = new Scalar(10, 255, 255);
        Core.inRange(hsvMat, minValues, maxValues, mask);

        Scalar minValues2 = new Scalar(170, 70, 50);
        Scalar maxValues2 = new Scalar(180, 255, 255);
        Core.inRange(hsvMat, minValues2, maxValues2, mask2);

        Core.bitwise_or(mask, mask2, combine);
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(combine, morphOutput, erodeElement);
        Imgproc.erode(morphOutput, morphOutput, erodeElement);

        Imgproc.dilate(morphOutput, morphOutput, dilateElement);
        Imgproc.dilate(morphOutput, morphOutput, dilateElement);
        // find contours
        Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
//        MatOfPoint2f matOfPoint2f;
//        for (int i = 0; i < contours.size(); i++) {
//            MatOfPoint pt = contours.get(i);
//            pt.fromList(pt.toList());
//
//            //Imgproc.approxPolyDP(pt, 3, true);
//            //Imgproc.boundingRect();
//        }
        // if any contour exist...
//        if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
//        {
//            // for each contour, display it in blue
//            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
//            {
//                Imgproc.drawContours(mat, contours, idx, new Scalar(250, 0, 0));
//            }
//        }

        MatOfPoint2f contour;
        List<MatOfPoint> polys = new ArrayList<MatOfPoint>();
        for (int i = 0; i < contours.size(); i++) {
            contour = new MatOfPoint2f(contours.get(i).toArray());
            MatOfPoint2f temp = new MatOfPoint2f();
            double peri = Imgproc.arcLength(contour, true);
            Imgproc.approxPolyDP(contour, temp, 0.02 * peri, true);
            RotatedRect rrect = Imgproc.minAreaRect(temp);

            //Rect brect = rrect.boundingRect();
            //Imgproc.rectangle(frame, brect, new Scalar(255,0,0), 7);

            Point[] rectPoints = new Point[4];
            rrect.points(rectPoints);

            List<MatOfPoint> list = new ArrayList<MatOfPoint>();
            list.add(
                    new MatOfPoint(rectPoints[0], rectPoints[1], rectPoints[2], rectPoints[3]
                    )
            );

            Imgproc.polylines(mat, list, true, new Scalar(0, 0, 0), 15);


            double centerX = (rectPoints[0].x + rectPoints[1].x + rectPoints[2].x + rectPoints[3].x) / 4;
            double centerY = (rectPoints[0].y + rectPoints[1].y + rectPoints[2].y + rectPoints[3].y) / 4;
            centerPoint = new Point(centerX, centerY);

            System.out.println("Point X " + centerX +  "  Y " + centerY);

            //Imgproc.circle(mat, centerPoint, 5, new Scalar(0, 0, 255), 5);

//        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
//        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
//            System.out.println("Mat Empty Ashi" + mat.empty());
            String filePath = new SimpleDateFormat("'/sdcard/FIRST/pic'yyyyMMddHHmmss'.png'").format(new Date());
//
            Date currentDate = new Date();
            long diffInMS = Math.abs(currentDate.getTime() - dtLastTimePic.getTime());
            if (!mat.empty()) {
                if (diffInMS > 2000) {
                    //   Imgcodecs.imwrite(filePath, mat);
                    //   System.out.println("Ashi Writing frame as" + filePath);
                    //   dtLastTimePic = new Date();
                }
            }

            mat.release();
            System.out.println("Got Frame");


        }

//
//        Mat detectColor (Mat srcImg){
//            Mat blurImg = new Mat();
//            Mat hsvImage = new Mat();
//            Mat color_range = new Mat();
//
//            //bluring image to filter small noises.
//            Imgproc.GaussianBlur(srcImg, blurImg, new Size(5, 5), 0);
//
//            //converting blured image from BGR to HSV
//            Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_BGR2HSV);
//
//            //filtering pixels based on given HSV color range
//            Core.inRange(hsvImage, new Scalar(0, 50, 50), new Scalar(5, 255, 255), color_range);
//
//            //filtering pixels based on given HSV color range
//            String filePath = "/sdcard/FIRST/color_rangeFile.png";
//            Imgcodecs.imwrite(filePath, color_range);
//            color_range.release();
//            System.out.println("Color Range");
//            return color_range;
//
//
//        }

        return centerPoint;
    }


}


