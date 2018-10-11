//
// Created by lalo on 11-10-18.
//


#include "geoproyectiva.h"

//// CONSTRUCTOR ////

geoproy::geoproy(bool start) {
    this -> start = true;
}


void geoproy::readCalibFile() {

    QString fileName;

    if(fileName == "")
        return;

//    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::READ);
//
//    FileNode calib = fs["calibration"];
//    FileNodeIterator it = calib.begin(), it_end = calib.end();
//
//// iterate through a sequence using FileNodeIterator
//    for( ; it != it_end; ++it ) {
//        cv::Mat h;
//        (*it)["Homography"] >> h;
//        calibration[(int)(*it)["Camera"]]  = h;
//    }
//    fs.release();
//    emit availableConfigurations();
//    if(cur_active > 0 && last.count(cur_active) > 0)
//        updateFrame(last[cur_active]);




}

