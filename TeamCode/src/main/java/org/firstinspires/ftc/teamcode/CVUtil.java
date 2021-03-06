package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
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


    public void onResume()
    {
        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, m_context, mLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }


    void updateFrame(Bitmap bm, Frame frame) {


        Image img = frame.getImage(0);
        Mat hsvMat = new Mat();
        Mat blurredMat = new Mat();
        Mat mask = new Mat();




        Mat mat = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, mat);


        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
        Imgproc.blur(mat, blurredMat, new Size(7, 7));
        Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_BGR2HSV);

        Scalar minValues = new Scalar(0, 0, 0);
        Scalar maxValues = new Scalar(180, 255, 255);
        Core.inRange(hsvMat, minValues, maxValues, mask);

//        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
//        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
        String filePath = "/sdcard/FIRST/mask.png";
        Imgcodecs.imwrite(filePath, mask);
        mat.release();
        System.out.println("Got Frame");

    }
    Mat detectColor(Mat srcImg) {
        Mat blurImg = new Mat();
        Mat hsvImage = new Mat();
        Mat color_range = new Mat();

        //bluring image to filter small noises.
        Imgproc.GaussianBlur(srcImg, blurImg, new Size(5,5),0);

        //converting blured image from BGR to HSV
        Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_BGR2HSV);

        //filtering pixels based on given HSV color range
        Core.inRange(hsvImage, new Scalar(0,50,50), new Scalar(5,255,255), color_range);

        //filtering pixels based on given HSV color range
        String filePath = "/sdcard/FIRST/color_rangeFile.png";
        Imgcodecs.imwrite(filePath, color_range);
        color_range.release();
        System.out.println("Color Range");
        return color_range;


    }




    }

