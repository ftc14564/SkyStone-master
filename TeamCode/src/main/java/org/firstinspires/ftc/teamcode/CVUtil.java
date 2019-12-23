package org.firstinspires.ftc.teamcode;

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

import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;
import android.graphics.Bitmap;

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

        Mat mat = new Mat(img.getHeight(), img.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, mat);

        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
        String filePath = "/sdcard/FIRST/rgbFile.png";
        Imgcodecs.imwrite(filePath, mat);
        mat.release();
        System.out.println("Got Frame");

    }
}
