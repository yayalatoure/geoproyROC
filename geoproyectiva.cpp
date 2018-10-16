//
// Created by lalo on 11-10-18.
//



#include <QtGui/QPainter>
#include "geoproyectiva.h"



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

void geoproy::genCalibPoints() {

    calibPoints.clear();
    cv::Point2f p;
    int step = 200;

    p.x = -step; p.y = -step;
    calibPoints[1] = p;
    p.x = 0;     p.y = -step;
    calibPoints[2] = p;
    p.x = step;  p.y = -step;
    calibPoints[3] = p;
    p.x = -step; p.y = 0;
    calibPoints[4] = p;
    p.x = 0;     p.y = 0;
    calibPoints[5] = p;
    p.x = step;  p.y = 0;
    calibPoints[6] = p;
    p.x = -step; p.y = step;
    calibPoints[7] = p;
    p.x = 0;     p.y = step;
    calibPoints[8] = p;
    p.x = step;  p.y = step;
    calibPoints[9] = p;


}

QPoint geoproy::transform(cv::Point2f p, cv::Mat &H) {
    cv::Mat pin(3, 1, CV_64FC1);
    pin.at<double>(0,0) = p.x;
    pin.at<double>(1,0) = p.y;
    pin.at<double>(2,0) = 1;

    cv::Mat pout = H*pin;

    return QPoint(int(pout.at<double>(0,0)/pout.at<double>(2,0)),
                  int(pout.at<double>(1,0)/pout.at<double>(2,0)));
}

void geoproy::drawRectangle(QPainter &pnt, Point2f &p, cv::Mat &H){

    int hside = 50;
    pnt.setPen(QColor(255,255,0));
    pnt.drawLine(transform(cv::Point2f(p.x-hside, p.y-hside), H),
                 transform(cv::Point2f(p.x+hside, p.y-hside), H));
    pnt.drawLine(transform(cv::Point2f(p.x+hside, p.y-hside), H),
                 transform(cv::Point2f(p.x+hside, p.y+hside), H));
    pnt.drawLine(transform(cv::Point2f(p.x+hside, p.y+hside), H),
                 transform(cv::Point2f(p.x-hside, p.y+hside), H));
    pnt.drawLine(transform(cv::Point2f(p.x-hside, p.y+hside), H),
                 transform(cv::Point2f(p.x-hside, p.y-hside), H));

}

void geoproy::addCalibPoints(QImage &image) {

    std::map<int, cv::Point2f>::iterator it, it_end = calibPoints.end();
    int index;
    QPoint p_im;
    QPainter pnt;
    cv::Mat &H = homography;
    pnt.begin(&image);

    for(it=calibPoints.begin(); it!=it_end; it++) {
        index = it->first;
        cv::Point2f &p = it->second;
        //Get center in image coordinates
        p_im = transform(p, H);

        //Draw circle
        pnt.setPen(QColor(255,0,0));
        pnt.drawEllipse(p_im, 3, 3);

        //Draw rectangle
        drawRectangle(pnt, p, H);

        //Draw text
//        pnt.setPen(QColor(0,255,0));
//        pnt.drawText(QPoint(p_im.x()+2,p_im.y()), QString::number(index));

    }
}

