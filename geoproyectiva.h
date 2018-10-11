//
// Created by lalo on 11-10-18.
//

#ifndef GEOPROYECTIVA_GEOPROYECTIVA_H
#define GEOPROYECTIVA_GEOPROYECTIVA_H


//#include <fstream>
//#include <iostream>
//#include <string>
//#include <QLabel>
//#include <QMessageBox>
#include <QFileDialog>



#include <opencv2/highgui/highgui.hpp>

class geoproy : public QWidget{

public:

    //// Constructor ////
    explicit  geoproy(bool start);

    //// Read Calib File ////
    void readCalibFile();


    //// Bool Atributes ////
    bool start;









};















#endif //GEOPROYECTIVA_GEOPROYECTIVA_H
