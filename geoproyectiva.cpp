//
// Created by lalo on 11-10-18.
//



#include <QtGui/QPainter>
#include "geoproyectiva.h"
#include "foot.h"



//// CONSTRUCTOR ////

geoproy::geoproy(bool start) {
    this -> start = true;
}

void geoproy::readCalibFile() {

//    //// Video Original
//    QString fileName = "/home/lalo/Desktop/Data_Videos/VideoOriginal/CALIB/default_calib1.yml";

//    //// UPLA Grabacion 1
//    QString fileName = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/default_calib.yml";

    //// UPLA Grabacion 2
    QString fileName = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/CALIB/default_calib_g2_1.yml";

    if(fileName == "")
        return;

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);

    FileNode calib = fs["calibration"];
    FileNodeIterator it = calib.begin(), it_end = calib.end();

    //// iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it ) {
        (*it)["Homography"] >> homography;
    }

    homographyInv = homography.inv();

    fs.release();

}

void geoproy::genCalibPointsSuelo() {

    calibPointsFloor.clear();
    cv::Point2f p;
    int step = 200;

    p.x = -step; p.y = -step;
    calibPointsFloor[1] = p;
    p.x = 0;     p.y = -step;
    calibPointsFloor[2] = p;
    p.x = step;  p.y = -step;
    calibPointsFloor[3] = p;
    p.x = -step; p.y = 0;
    calibPointsFloor[4] = p;
    p.x = 0;     p.y = 0;
    calibPointsFloor[5] = p;
    p.x = step;  p.y = 0;
    calibPointsFloor[6] = p;
    p.x = -step; p.y = step;
    calibPointsFloor[7] = p;
    p.x = 0;     p.y = step;
    calibPointsFloor[8] = p;
    p.x = step;  p.y = step;
    calibPointsFloor[9] = p;

    genCalibPointsImage();

}

void geoproy::genCalibPointsImage(){

    std::map<int, cv::Point2f>::iterator it, it_end = calibPointsFloor.end();
    int index;

    for(it=calibPointsFloor.begin(); it!=it_end; it++) {
        index = it->first;
        cv::Point2f &p = it->second;
        calibPointsImage[index] = transform(p, homography);
    }

}

void geoproy::genCalibPointsCorner() {

    calibPointsCornerFloor.clear();
    cv::Point2f p;
    int step = 350;

    p.x = -step; p.y = -step;
    calibPointsCornerFloor[1] = p;
    p.x = step; p.y = -step;
    calibPointsCornerFloor[2] = p;
    p.x = step;  p.y = step;
    calibPointsCornerFloor[3] = p;
    p.x = -step; p.y = step;
    calibPointsCornerFloor[4] = p;

    for (int i = 1; i <= 4; ++i) {
        calibPointsCornerImage[i] = transform(calibPointsCornerFloor[i], homography);
        roiConvexPoly.push_back(calibPointsCornerImage[i]);
    }

}


cv::Point geoproy::transform(cv::Point2f p, cv::Mat &H) {

    cv::Mat pin(3, 1, CV_64FC1); // NOLINT
    pin.at<double>(0,0) = p.x;
    pin.at<double>(1,0) = p.y;
    pin.at<double>(2,0) = 1;

    cv::Mat res = H*pin;
    cv::Point pout;

    pout.x = int(res.at<double>(0,0)/res.at<double>(2,0));
    pout.y = int(res.at<double>(1,0)/res.at<double>(2,0));

    return pout;
}

void geoproy::drawRectangleRed(QPainter &pnt, Point2f &p, cv::Mat &H){

    cv::Point point1, point2;
    int hside = 50;
    pnt.setPen(QColor(255,255,0));

    point1 = transform(cv::Point2f(p.x-hside, p.y-hside), H);
    point2 = transform(cv::Point2f(p.x+hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transform(cv::Point2f(p.x+hside, p.y-hside), H);
    point2 = transform(cv::Point2f(p.x+hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transform(cv::Point2f(p.x+hside, p.y+hside), H);
    point2 = transform(cv::Point2f(p.x-hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transform(cv::Point2f(p.x-hside, p.y+hside), H);
    point2 = transform(cv::Point2f(p.x-hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

}

void geoproy::drawRectangleBlue(QPainter &pnt, cv::Mat &H){

    cv::Point point1, point2;
    pnt.setPen(QColor(255,255,0));

    point1 = calibPointsCornerImage[1];
    point2 = calibPointsCornerImage[2];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[2];
    point2 = calibPointsCornerImage[3];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[3];
    point2 = calibPointsCornerImage[4];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[4];
    point2 = calibPointsCornerImage[1];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

}

void geoproy::addCalibPoints(QImage &image) {

    std::map<int, cv::Point2f>::iterator it1, it1_end = calibPointsFloor.end();
    int index;
    QPoint p_im;
    QPainter pnt;
    pnt.begin(&image);
    cv::Point pout;
    cv::Mat &H = homography;

    for(it1=calibPointsFloor.begin(); it1!=it1_end; it1++) {
        index = it1->first;
        cv::Point2f &p = it1->second;

        pout = transform(p, H);
        p_im = QPoint(pout.x, pout.y);

        //Draw circle
        pnt.setPen(QColor(255,0,0));
        pnt.drawEllipse(p_im, 3, 3);

        //Draw rectangle
        drawRectangleRed(pnt, p, H);

        //Draw text
        pnt.setPen(QColor(0,255,0));
        pnt.drawText(QPoint(p_im.x() + 5, p_im.y()), QString::number(index));

    }

    drawRectangleBlue(pnt, H);

    pnt.end();
}


