

#include "genroc.h"


//// CONSTRUCTOR ////
genroc::genroc(bool start) {
    this -> start = true;
}

void genroc::calibVideoSelector(int numVideo){

    switch(numVideo) {
        case 1 :
            numCalibVideo = 1;
            break;
        case 2 :
            numCalibVideo = 2;
            break;
        case 3 :
            numCalibVideo = 2;
            break;
        case 4 :
            numCalibVideo = 2;
            break;
        case 5 :
            numCalibVideo = 2;
            break;
        case 6 :
            numCalibVideo = 3;
            break;
        case 7 :
            numCalibVideo = 3;
            break;
        case 8 :
            numCalibVideo = 4;
            break;
        case 9 :
            numCalibVideo = 4;
            break;
        default :
            break;
    }

}

void genroc::getVideo(int numVideo) {

    pathVideos = "/home/lalo/Desktop/Data/Videos/Video";
    pathCalib  = "/home/lalo/Desktop/Data/Calib/Video";

    calibVideoSelector(numVideo);

    path_test = pathVideos + to_string(numVideo) + "/*.jpg";
    path_cal  = pathCalib + to_string(numCalibVideo)+"/*.jpg";
    calfileName = pathCalib + to_string(numCalibVideo) + "/default_calib.yml";

    glob(path_test, filenames_test);
    glob(path_cal , filenames_cal);

    pos = filenames_test[count_test].find(".jpg");


}

void genroc::algorithm() {

    foot Foot(false);
    geoproy geoproyTest(true);

    geoproyTest.readCalibFile(calfileName);
    geoproyTest.genCalibPointsSuelo();
    geoproyTest.genCalibPointsCorner();
    cout << "Homography: \n" << geoproyTest.homography << "\n" << endl;
    cout << "Homography Inv: \n" << geoproyTest.homographyInv << "\n" << endl;
    geoproyTest.generateSequence(Foot.seed);
    geoproyTest.playsToObjetives();

    Foot.maskConvexPoly(geoproyTest);
    Foot.kalmanInit(Foot.Right);
    Foot.kalmanInit(Foot.Left);

    cout << "\n" << endl;
    cout << "Start Algorithm" << endl;
    cout << "\n" << endl;

    while(ch != 'q' && ch != 'Q') {

        //// Transfer Frame Structure ////
        Foot.frameAnt = Foot.frameAct;

        ////////// Frame Acquisition /////////
        if (count_cal < limit) {
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = imread(filenames_cal[count_cal], CV_LOAD_IMAGE_COLOR);
            substring = filenames_cal[count_cal].substr(pos - digits);
            //cout << substring << endl;

        } else {
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.start = true;
            cout << substring << endl;
            Foot.frame = substring;

        }

        ///// Algoritmo /////

        if (Foot.frameAct.processFrame.data && Foot.start) {

            //// Low Step Flag ////
            Foot.step_R = false;
            Foot.step_L = false;

            //// Clone process frame to result frame ////
            Foot.frameAct.resultFrame = Foot.frameAct.processFrame.clone();

            Foot.segmentation();
            Foot.getLowerBox();

            Foot.getFeetBoxes(geoproyTest);

            Foot.occlusion = bool(Foot.frameAct.footBoxes.size() <= 1);

            //// Kalman Filter ////
            Foot.kalmanPredict(Foot.Right, dT);
            Foot.kalmanPredict(Foot.Left, dT);

            //// Kalman Update ////
            Foot.kalmanUpdate(Foot.Right);
            Foot.kalmanUpdate(Foot.Left);

            //// Measure Error ////
            Foot.measureError1Np(Foot.Right);
            Foot.measureError1Np(Foot.Left);

            //// Kalman Reset Step ////
            Foot.kalmanResetStep(Foot.Right);
            Foot.kalmanResetStep(Foot.Left);


            Foot.askObjetives(geoproyTest);
            if (!Foot.stop){
                //// Matching Objetives State Machine////
                Foot.stateMachine(geoproyTest);
            }
            Foot.frameAct.processFrame.copyTo(Foot.frameAct.resultFrame);


            //Foot.drawingResults();

            qedit = QImage((uchar*) Foot.frameAct.resultFrame.data, Foot.frameAct.resultFrame.cols,
                    Foot.frameAct.resultFrame.rows, int(Foot.frameAct.resultFrame.step), QImage::Format_RGB888);
            geoproyTest.addCalibPoints(qedit);
            geoproyTest.paintCircles(qedit);
            geopro = cv::Mat(qedit.height(), qedit.width(), CV_8UC3, (uchar*) qedit.bits(),
                             static_cast<size_t>(qedit.bytesPerLine())); // NOLINT

        } else{
            if(Foot.frameAct.processFrame.data){
                Foot.segmentation();
            }
        }

        if (Foot.start && (Foot.frameAct.resultFrame.data)) {
            cv::imshow("frame", Foot.frameAct.resultFrame);
            ch = char(cv::waitKey(0));
        }

        count_cal++;
        count_test++;

    }
}


