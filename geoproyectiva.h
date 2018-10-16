//
// Created by lalo on 11-10-18.
//

#ifndef GEOPROYECTIVA_GEOPROYECTIVA_H
#define GEOPROYECTIVA_GEOPROYECTIVA_H


#include <fstream>
#include <iostream>
#include <string>
#include <QLabel>
#include <QMessageBox>
#include <QFileDialog>


#include <opencv/cv.h>

using namespace std;
using namespace cv;


class geoproy {

public:

    //// Constructor ////
    explicit geoproy(bool start);

    //// Read Calib File ////
    void readCalibFile();
    //// Generate Scene Calibpoints ////
    void genCalibPoints();
    //// Add CalibPoints to CalibImage ////
    void addCalibPoints(QImage &image);
    //// Transform Point from Image to Scene ////
    static QPoint transform(Point2f p, cv::Mat &H);
    //// Draw Rectangle Round CalibPoints////
    void drawRectangle(QPainter &pnt, Point2f &p, cv::Mat &H);




    //// Bool Atributes ////
    bool start;
    //// Mat Atributes ////
    Mat homography;
    Mat frameAct;
    //// Points Vector Atributes ////
    std::map<int, cv::Point2f> calibPoints;








};















#endif //GEOPROYECTIVA_GEOPROYECTIVA_H
