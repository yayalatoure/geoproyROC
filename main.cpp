
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
    cv::Mat to_write(480, 640, CV_8UC3, Scalar(255,255,255));

    QGuiApplication a(argc, argv);

    foot Foot(false);
    geoproy geoproyTest(true);


    Foot.playerName = "Fabian";
    Foot.pasadaCali = "Pasada2";
    Foot.pasadaTest = "Pasada2";
    Foot.seed = 2025060938;
    Foot.limit = 0;
    Foot.logCSVInit();


    //// UPLA Grabacion 3 Player Videos
    string path_cal  = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion3/CALIB/"+Foot.pasadaCali+"/*.jpg";
    QString fileName = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion3/CALIB/"+Foot.pasadaTest+"/default_calib.yml";
    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion3/DATA/"+Foot.pasadaTest.toStdString()+"/"+
                       Foot.playerName+"/*.jpg";
    int count_test = 0, count_cal = 0, limit = 10;

    /*
    cv::VideoWriter out;
    string videoName = "/home/lalo/Dropbox/NeuroCoachVideos/Video3-Hector.avi";
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    bool isColor = (to_write.type() == CV_8UC3);
    out.open(videoName, codec, 25, to_write.size(), isColor);
    if (!out.isOpened()) {
        cout << "Could not open the output video file for write\n";
        return 0;
    }
    */

    vector<String> filenames_cal, filenames_test;

    glob(path_test, filenames_test);
    glob(path_cal , filenames_cal);

    int digits = 5;
    size_t pos = filenames_test[count_test].find(".jpg");
    string substring;

    char ch = 0;
    int dT = 1;

    geoproyTest.readCalibFile(fileName);
    geoproyTest.genCalibPointsSuelo();
    geoproyTest.genCalibPointsCorner();

    cout << "Homography: \n" << geoproyTest.homography << "\n" << endl;
    cout << "Homography Inv: \n" << geoproyTest.homographyInv << "\n" << endl;

    geoproyTest.generateSequence(Foot.seed);
    geoproyTest.playsToObjetives();

    Foot.maskConvexPoly(geoproyTest);
    Foot.kalmanInit(Foot.Right);
    Foot.kalmanInit(Foot.Left);

    QImage edit;
    cv::Mat geopro, img;

    cout << "\n" << endl;
    cout << "Start Algorithm" << endl;
    cout << "\n" << endl;


    while(ch != 'q' && ch != 'Q') {

        //// Transfer Frame Structure ////
        Foot.frameAnt = Foot.frameAct;

        ////////// Frame Acquisition /////////
        if (count_cal < limit) {
            img_cal = imread(filenames_cal[count_cal], CV_LOAD_IMAGE_COLOR);
            substring = filenames_cal[count_cal].substr(pos - digits);
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = img_cal;
            //cout << substring << endl;

        } else {
            img_test = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = img_test;
            Foot.start = true;
            //cout << substring << endl;
            Foot.frame = substring;


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

            Foot.occlusion = bool(Foot.frameAct.footBoxes.size() <= 1);

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

            //// debug
            Foot.askObjetives(geoproyTest);

            if (!Foot.stop){
                //// Matching Objetives State Machine////
                Foot.stateMachine(geoproyTest);
            }

            Foot.frameAct.processFrame.copyTo(Foot.frameAct.resultFrame);

            Foot.drawingResults();

            img = Foot.frameAct.resultFrame.clone();
            edit = QImage((uchar*) img.data, img.cols, img.rows, int(img.step), QImage::Format_RGB888);
            geoproyTest.addCalibPoints(edit);
            geopro = cv::Mat(edit.height(), edit.width(), CV_8UC3, (uchar*) edit.bits(), static_cast<size_t>(edit.bytesPerLine())); // NOLINT

            //// COUTS ////
            /*
            cout << "Zone?: " << Foot.platformZone << endl;
            cout << "Size: " << Foot.frameAct.footBoxes.size() << endl;
            cout << "StepR : " << Foot.step_R << endl;
            cout << "StepL : " << Foot.step_L << endl;
            cout << "CountCenterOut: " << Foot.countCenterOut << endl;
            cout << "FoundMatchR: " << Foot.foundMatchR << endl;
            cout << "FoundMatchL: " << Foot.foundMatchL << endl;
            cout << "countCenterOut: " << Foot.countCenterOut << endl;
            cout << "\n" << endl;
            cout << "Occlusion?: " << Foot.occlusion << endl;
            cout << "Width R: " << Foot.frameAct.footBoxes[Foot.Right].width << endl;
            cout << "Width L: " << Foot.frameAct.footBoxes[Foot.Left].width << endl;
            cout << "centerFlagWasOut: " << Foot.centerFlagWasOut << endl;
            cout << "centerFlagIsIn: " << Foot.centerFlagIsIn << endl;
            cout << "betweenFromObjet: " << Foot.betweenFromObjet << endl;
            */

        } else{
            if(Foot.frameAct.processFrame.data){
                Foot.segmentation();
            }
        }

        if (Foot.start && (Foot.frameAct.resultFrame.data)) {
//            cv::imshow("frameAct", Foot.frameAct.resultFrame);
            cv::imshow("Segment", Foot.frameAct.segmentedFrame);
            cv::imshow("geoProy", geopro);

            //out << geopro;

            ch = char(cv::waitKey(0));

        }
        count_cal++;
        count_test++;

    }

    return 0;

}