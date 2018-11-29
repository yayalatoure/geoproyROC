

#include "genroc.h"
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
    cv::Mat to_write(480, 640, CV_8UC3, Scalar(255,255,255)); // NOLINT

    QGuiApplication a(argc, argv);

    genroc genROC(true);


    genROC.getVideo(1);
    genROC.algorithm();

    return 0;

}