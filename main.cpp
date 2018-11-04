
#include "foot.h"
#include "geoproyectiva.h"
#include <QApplication>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv/cv.h>

#include <QRect>
#include <QDir>
#include <QLabel>
#include <QPainter>
#include <QtGui>
#include <QGuiApplication>


using namespace std;
using namespace cv;

int main(int argc, char *argv[]){

    cv::Mat img_cal, img_test;

    QGuiApplication a(argc, argv);

//    //// Video Original
//    string path_cal  = "/home/lalo/Dropbox/Proyecto IPD441/Data/Videos/1_CAMARA/CALIBRACION01/*.jpg";
//    string path_test = "/home/lalo/Dropbox/Proyecto IPD441/Data/Videos/1_CAMARA/TEST01/*.jpg";
//    int count_test = 195+145, count_cal = 0, limit = 150-145;


    //// UPLA Grabacion 1
    string path_cal  = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/CAL_Test1/*.jpg";
//    //// Player2
//    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/Player2/*.jpg";
//    int count_test = 1210, count_cal = 0, limit = 5;
    //// Player3
    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/Player3/*.jpg";
    int count_test = 165+145, count_cal = 0, limit = 5;
    int seed = 463094935;


//    //// UPLA Grabacion 2
//    string path_cal  = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/CALIB/CAL1/*.jpg";
//    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/DATA1/Data1Player5/*.jpg";
//    int count_test = 582-142, count_cal = 0, limit = 10;


    vector<String> filenames_cal, filenames_test;

    glob(path_test, filenames_test);
    glob(path_cal , filenames_cal);

    int digits = 5;
    size_t pos = filenames_test[count_test].find(".jpg");
    string substring;

    char ch = 0;
    int dT = 1;
    foot Foot(false);
    geoproy geoproyTest(true);

    geoproyTest.readCalibFile();
    geoproyTest.genCalibPointsSuelo();
    geoproyTest.genCalibPointsCorner();

    cout << "Homography: \n" << geoproyTest.homography << "\n" << endl;
    cout << "Homography Inv: \n" << geoproyTest.homographyInv << "\n" << endl;

    geoproyTest.generateSequence(seed);

    Foot.maskConvexPoly(geoproyTest);
    Foot.kalmanInit(Foot.Right);
    Foot.kalmanInit(Foot.Left);

    QImage edit;
    cv::Mat geopro, img;

    geoproyTest.paintObjetive(1);



    while(ch != 'q' && ch != 'Q') {

        //// Transfer Frame Structure ////
        Foot.frameAnt = Foot.frameAct;

        ////////// Frame Acquisition /////////
        if (count_cal < limit) {
            img_cal = imread(filenames_cal[count_cal], CV_LOAD_IMAGE_COLOR);
            img_test = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = img_cal;

            cout << substring << "\n" << endl;

        } else {
            img_test = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = img_test;
            Foot.start = true;

            cout << substring << "\n" << endl;

        }

        ///// Algoritmo /////

        if (Foot.frameAct.processFrame.data && Foot.start) {

            //// Low Step Flag ////
            Foot.step_R = false;
            Foot.step_L = false;

            //// Clone process frame to result frame ////
            Foot.frameAct.resultFrame = Foot.frameAct.processFrame.clone();

            Foot.segmentation();
            Foot.getLowerBox();

            Foot.getFeetBoxes(geoproyTest);

            Foot.occlusion = bool(Foot.frameAct.footBoxes.size() <= 2);

            cout << "Occlussion?: " << Foot.occlusion << endl;

            //// Measure Foot ////
            Foot.measureFoot(Foot.Right);
            Foot.measureFoot(Foot.Left);

            //// Kalman Filter ////
            Foot.kalmanPredict(Foot.Right, dT);
            Foot.kalmanPredict(Foot.Left, dT);

            //// Kalman Update ////
            Foot.kalmanUpdate(Foot.Right);
            Foot.kalmanUpdate(Foot.Left);

            //// Measure Error ////
            Foot.measureError1Np(Foot.Right);
            Foot.measureError1Np(Foot.Left);

            //// Kalman Reset Step ////
            Foot.kalmanResetStep(Foot.Right);
            Foot.kalmanResetStep(Foot.Left);


            img = Foot.frameAct.processFrame.clone();
            edit = QImage((uchar*) img.data, img.cols, img.rows, int(img.step), QImage::Format_RGB888);
            geoproyTest.addCalibPoints(edit);
            geopro = cv::Mat(edit.height(), edit.width(), CV_8UC3, (uchar*)edit.bits(), static_cast<size_t>(edit.bytesPerLine())); // NOLINT

            Foot.frameAct.processFrame.copyTo(Foot.frameAct.resultFrame);

            Foot.drawingResults();


//            cout << "Zone?: " << Foot.platformZone << endl;
//            cout << "Size: " << Foot.frameAct.footBoxes.size() << endl;
//            cout << "Occlusion?: " << Foot.occlusion << endl;


        } else{
            if(Foot.frameAct.processFrame.data){
                Foot.segmentation();
            }
        }





        if (Foot.start && (Foot.frameAct.resultFrame.data)) {

            cv::imshow("frameAct", Foot.frameAct.resultFrame);
            cv::imshow("Segment", Foot.frameAct.segmentedFrame);

            cv::imshow("geoProy", geopro);

        }



        count_cal++;
        count_test++;
        ch = char(cv::waitKey(0));



    }





    return 0;

}