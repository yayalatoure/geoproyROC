
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

    //// Images Directories
    string path_cal  = "/home/lalo/Desktop/Data_Videos/CAL_Test1/*.jpg";
    string path_test = "/home/lalo/Desktop/Data_Videos/Player3/*.jpg";

    //// Video Original
    int count_test = 195+145, count_cal = 0, limit = 150-145;

    vector<String> filenames_cal, filenames_test;

    glob(path_test, filenames_test);
    glob(path_cal , filenames_cal);

    int digits = 5;
    size_t pos = filenames_test[count_test].find(".jpg");
    string substring;

    char ch = 0;
    foot Foot(false);
    geoproy geoproyTest(true);

    geoproyTest.readCalibFile();
    geoproyTest.genCalibPointsSuelo();
    geoproyTest.genCalibPointsImage();
    geoproyTest.genCalibPointsCorner();


    cout << geoproyTest.homography << "\n" << endl;


    QImage edit;
    cv::Mat geopro, img;

    while(ch != 'q' && ch != 'Q') {

        ////////// Frame Acquisition /////////
        if (count_cal < limit) {
            img_cal = imread(filenames_cal[count_cal], CV_LOAD_IMAGE_COLOR);
            img_test = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frameAct.processFrame = img_cal;
            Foot.start = true;

            cout << substring << "\n" << endl;

        } else {
            img_test = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frameAct.processFrame = img_test;
            Foot.start = true;

            cout << substring << "\n" << endl;

        }

        ///// Algoritmo /////

        if (Foot.frameAct.processFrame.data) {

            Foot.segmentation();
            Foot.maskConvexPoly(geoproyTest);


            img = Foot.frameAct.processFrame.clone();
            edit = QImage((uchar*) img.data, img.cols, img.rows, int(img.step), QImage::Format_RGB888);
            geoproyTest.addCalibPoints(edit);
            geopro = cv::Mat(edit.height(), edit.width(), CV_8UC3, (uchar*)edit.bits(), static_cast<size_t>(edit.bytesPerLine()));




        }







        if (Foot.start && (Foot.frameAct.processFrame.data)) {

            cv::imshow("frameAct", Foot.frameAct.processFrame);
//            cv::imshow("Segment", Foot.frameAct.segmentedFrame);
            cv::imshow("geoProy", geopro);
            cv::imshow("segmConvexPoly", geoproyTest.maskConvexPoly);



        }

        count_cal++;
        count_test++;
        ch = char(cv::waitKey(0));

    }























    std::cout << "Hello, World!" << std::endl;
    return 0;

}