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


    //// Bool Atributes ////
    bool start;

    //// Mat Atributes ////
    Mat homography;








};















#endif //GEOPROYECTIVA_GEOPROYECTIVA_H
