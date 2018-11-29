

#ifndef GEOPROYECTIVA_GENROC_H
#define GEOPROYECTIVA_GENROC_H

#include "foot.h"
#include "geoproyectiva.h"


using namespace std;
using namespace cv;



class genroc {

public:

    //// Constructor ////
    explicit genroc(bool start);

    //// PUBLIC METHODS ////
    void calibVideoSelector(int numVideo);
    void getVideo(int numVideo);
    void algorithm();
    void videoWriter();


public:

    //// ATRIBUTES ////

    //// Bool Atributes ////
    bool start;

    //// Video Get Atributes ////
    int dT = 1;
    int limit = 10;
    int digits = 5;
    int count_cal = 0;
    int count_test = 0;
    int numCalibVideo = 0;

    char ch = 0;
    size_t pos;

    string pathVideos;
    string pathCalib;
    string path_cal;
    string path_test;
    string calfileName;
    vector<String> filenames_cal, filenames_test;

    string substring;


    //// Imshow Atributes ////
    Mat geopro;
    QImage qedit;






    //// Points Vector Atributes ////




};



#endif //GEOPROYECTIVA_GENROC_H
