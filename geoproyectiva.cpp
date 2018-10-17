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

    QString fileName = "/home/lalo/Desktop/Data_Videos/default_calib.yml";

    if(fileName == "")
        return;

    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);

    FileNode calib = fs["calibration"];
    FileNodeIterator it = calib.begin(), it_end = calib.end();

    //// iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it ) {
        (*it)["Homography"] >> homography;
    }

    fs.release();

}

void geoproy::genCalibPointsSuelo() {

    calibPointsSuelo.clear();
    cv::Point2f p;
    int step = 200;

    p.x = -step; p.y = -step;
    calibPointsSuelo[1] = p;
    p.x = 0;     p.y = -step;
    calibPointsSuelo[2] = p;
    p.x = step;  p.y = -step;
    calibPointsSuelo[3] = p;
    p.x = -step; p.y = 0;
    calibPointsSuelo[4] = p;
    p.x = 0;     p.y = 0;
    calibPointsSuelo[5] = p;
    p.x = step;  p.y = 0;
    calibPointsSuelo[6] = p;
    p.x = -step; p.y = step;
    calibPointsSuelo[7] = p;
    p.x = 0;     p.y = step;
    calibPointsSuelo[8] = p;
    p.x = step;  p.y = step;
    calibPointsSuelo[9] = p;

}

void geoproy::genCalibPointsCorner() {

    calibPointsCornerSuelo.clear();
    cv::Point2f p;
    int step = 350;

    p.x = -step; p.y = -step;
    calibPointsCornerSuelo[1] = p;
    p.x = -step; p.y = step;
    calibPointsCornerSuelo[2] = p;
    p.x = step;  p.y = -step;
    calibPointsCornerSuelo[3] = p;
    p.x = step; p.y = step;
    calibPointsCornerSuelo[4] = p;

    for (int i = 1; i <= 4; ++i) {
        calibPointsCornerImage[i] = transform(calibPointsCornerSuelo[i], homography);
    }

}

void geoproy::genCalibPointsImage(){

    std::map<int, cv::Point2f>::iterator it, it_end = calibPointsSuelo.end();
    int index;

    for(it=calibPointsSuelo.begin(); it!=it_end; it++) {
        index = it->first;
        cv::Point2f &p = it->second;
        calibPointsImage[index] = transform(p, homography);
    }

}

cv::Point geoproy::transform(cv::Point2f p, cv::Mat &H) {

    cv::Mat pin(3, 1, CV_64FC1);
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
    cv::Point p;
    p.x = 0, p.y = 0;
    int hside = 350;
    pnt.setPen(QColor(255,255,0));

    point1 = transform(cv::Point2f(p.x-hside, p.y-hside), H);
    point2 = transform(cv::Point2f(p.x+hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

//    point1 = transform(cv::Point2f(p.x+hside, p.y-hside), H);
//    point2 = transform(cv::Point2f(p.x+hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

//    point1 = transform(cv::Point2f(p.x+hside, p.y+hside), H);
//    point2 = transform(cv::Point2f(p.x-hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

//    point1 = transform(cv::Point2f(p.x-hside, p.y+hside), H);
//    point2 = transform(cv::Point2f(p.x-hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

}

void geoproy::addCalibPoints(QImage &image) {

    std::map<int, cv::Point2f>::iterator it1, it1_end = calibPointsSuelo.end();
    std::map<int, cv::Point2f>::iterator it2, it2_end = calibPointsCorner.end();
    int index;
    QPoint p_im;
    QPainter pnt;
    pnt.begin(&image);
    cv::Point pout;
    cv::Mat &H = homography;

    for(it1=calibPointsSuelo.begin(); it1!=it1_end; it1++) {
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

    /*
    for(it2=calibPointsCorner.begin(); it2!=it2_end; it2++) {

        index = it2->first;
        cv::Point2f &pc = it2->second;

        pout = transform(pc, H);
        p_im = QPoint(pout.x, pout.y);

        //Draw circle
        pnt.setPen(QColor(255,0,0));
        pnt.drawEllipse(p_im, 3, 3);

        //Draw text
        pnt.setPen(QColor(0,255,0));
        pnt.drawText(QPoint(p_im.x() + 5, p_im.y()), QString::number(index));

    }
     */

    drawRectangleBlue(pnt, H);

    pnt.end();
}


