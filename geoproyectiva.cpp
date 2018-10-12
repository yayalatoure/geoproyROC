//
// Created by lalo on 11-10-18.
//



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

