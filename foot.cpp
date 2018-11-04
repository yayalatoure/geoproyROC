//
// Created by lalo on 30-08-18.
//

#include "foot.h"


//// CONSTRUCTOR ////
foot::foot(bool start) {
    this -> start = false;
    Reset_R = false;
    Reset_L = false;
    step_R = false;
    step_L = false;
    occlusion = false;
}

//// COLORS ////
cv::Scalar foot::blue(255, 0, 0); // NOLINT
cv::Scalar foot::green(0, 255, 0); // NOLINT
cv::Scalar foot::red(0, 0, 255); // NOLINT

cv::Scalar foot::cyan(255, 255, 0); // NOLINT
cv::Scalar foot::ivory(240, 255, 255); // NOLINT
cv::Scalar foot::blueviolet(226, 43, 138); // NOLINT
cv::Scalar foot::orange(0, 165, 255);


//// Euclidean Distance ////
double foot::distance(cv::Point center_kalman, cv::Point center_measured) {
    double dx = 0, dy = 0, result = 0;
    dx = pow(center_kalman.x - center_measured.x, 2);
    dy = pow(center_kalman.y - center_measured.y, 2);
    result = sqrt(dx + dy);
    return result;
}

//// SEGMENTATION AND FOOT BOXES ////

//// Draw Foot Rectangles from Measurement ////
void foot::paintRectangles(cv::Mat &img, std::map<int, cv::Rect> &bboxes, cv::Scalar color){

    std::map<int, cv::Rect>::iterator it, it_end = bboxes.end();
    int i = 0;
    for(it = bboxes.begin(); it != it_end; it++) {
        i += 1;
        cv::rectangle(img, it->second, color, 2);
//        if (i == 2)
//            break;
    }

}

void foot::paintRectanglesVector(vector<Rect> &vectorBoxes, cv::Scalar color){

    for(int i = 0; i < vectorBoxes.size(); i++) {
        cv::rectangle(frameAct.processFrame, vectorBoxes[i], color, 2);
//        if (i == 1)
//            break;
    }

}

/*
//// Get Foot Boxes from Blobs and Labels ////
void foot::getFeet(cv::Mat fg, std::map<int, cv::Rect> &bboxes, cv::Mat labels, cv::Mat labels2, std::map<int, cv::Rect> &fboxes){

    // Selecciona la región conectada más grande
    int Direc = 0, biggestblob = 1;
    string Direccion;

    getBlobsBoxes(labels, bboxes); // NOLINT

    for(unsigned int j=0; j < bboxes.size(); j++){
        if(bboxes[j].area() >= bboxes[biggestblob].area()) biggestblob = j;
    }

    // Crea una ROI en la parte inferior del jugador para visualizar sólo los
    // pies y eliminar el resto del análisis.

    Rect ROI;
    ROI.x = bboxes[biggestblob].x;
    ROI.y = int( bboxes[biggestblob].y + bboxes[biggestblob].height*0.8);
    ROI.height = int(bboxes[biggestblob].height*0.5);                       //// porcentaje caja inferior
    ROI.width = bboxes[biggestblob].width;

    Mat mask = Mat::zeros(fg.size(), CV_8U);
    rectangle(mask, ROI, Scalar(255), CV_FILLED);
    Mat fgROI = Mat::zeros(fg.size(), CV_8U);

    // copia fg a fgROI donde mask es distinto de cero.
    fg.copyTo(fgROI, mask);
    // aplica componentes conectados otra vez.
    cv::connectedComponents(fgROI, labels2, 8, CV_32S);
    getBlobsBoxes(labels2, fboxes);

}
*/


//// Convex Polygon Platform Mask ////
void foot::maskConvexPoly(geoproy GeoProy){

    Mat mask = Mat(rowsIm, colsIm, CV_8UC1, Scalar(0)); // NOLINT
    approxPolyDP(GeoProy.roiConvexPoly, GeoProy.roiConvexPoly, 1.0, true);
    fillConvexPoly(mask, &GeoProy.roiConvexPoly[0], (int)GeoProy.roiConvexPoly.size(), 255, 8, 0);
    frameAct.maskConvexPoly = mask.clone();

}

//// Segmentation and Foot Boxes ////
void foot::segmentation(){

    cv::Mat processMasked, foreGround, labels, stats, centroids;

    double backgroundRatio = 0.6;
    double learningRate = 0.004; ////0.005
    double varThreshold = 250; //// 210
    int    nmixtures = 3;
    int    history = 200; ////150

    static cv::Ptr<cv::BackgroundSubtractorMOG2> mog = cv::createBackgroundSubtractorMOG2(history, varThreshold, true);
    mog->setNMixtures(nmixtures);
    mog->setBackgroundRatio(backgroundRatio);
    mog->setShadowValue(0);
    mog->setShadowThreshold(0.42);
    mog->setDetectShadows(1);


    //// Start Segmentation ////
    //// Convex Polygon Mask ////
    frameAct.processFrame.copyTo(processMasked, frameAct.maskConvexPoly);

    mog->apply(processMasked, foreGround, 2*learningRate);
    cv::dilate(foreGround, foreGround, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5))); ////(4,6)
    cv::erode(foreGround, foreGround, cv::getStructuringElement(cv::MORPH_RECT,  cv::Size(3, 3))); ////(4,6)
    cv::connectedComponentsWithStats(foreGround, labels, stats, centroids, 8, CV_32S);

    frameAct.segmentedFrame  =  foreGround.clone();
    frameAct.labelsFrame = labels.clone();

}

//// Order to BoxesVector ////
struct byArea {
    bool operator () (const Rect &a, const Rect &b) {
        return a.width*a.height > b.width*b.height ;
    }
};

//// Get Bigger Blobs of Segmentation Image ////
void foot::getBlobsBoxes(cv::Mat labels, std::map<int, cv::Rect> &bboxes) {

    int ro = labels.rows, co = labels.cols;
    int label, x, y;

    bboxes.clear();

    for (int j = 0; j < ro; ++j) {
        for (int i = 0; i < co; ++i) {
            label = labels.at<int>(j, i);
            if (label > 0) {                    // Not Background?
                if (bboxes.count(label) == 0) { // New label
                    cv::Rect r(i, j, 1, 1);
                    bboxes[label] = r;
                } else {                       // Update rect
                    cv::Rect &r = bboxes[label];
                    x = r.x + r.width - 1;
                    y = r.y + r.height - 1;
                    if (i < r.x) r.x = i;
                    if (i > x) x = i;
                    if (j < r.y) r.y = j;
                    if (j > y) y = j;
                    r.width = x - r.x + 1;
                    r.height = y - r.y + 1;
                }
            }
        }
    }

}

void foot::orderVectorBoxes(std::map<int, cv::Rect> &bboxes, vector<Rect> &vectorBoxes){

    for(unsigned int j=0; j < bboxes.size(); j++){   //NOLINT
        vectorBoxes.push_back(bboxes[j]);
    }

    std::sort(vectorBoxes.begin(), vectorBoxes.end(), byArea());
    /*
    cout << "footBoxesVector.size: " << vectorBoxes.size() << endl;
    cout << "Areas ordenadas: " << endl;
    for (int i = 0; i < vectorBoxes.size() ; ++i) {
        cout << vectorBoxes[i].area() << endl;
    }
    */

};

void foot::findLowerBox(){

    Rect ROI (0, 0, 1, 1);
    frameAct.segmLowerBox = ROI;

    cv::Point point1, point2;
    double result;
    int threshold = 50;

    if(frameAct.segmLowerBoxesVector.size()>1) {
        point1.x = frameAct.segmLowerBoxesVector[0].x + frameAct.segmLowerBoxesVector[0].width/2;
        point1.y = frameAct.segmLowerBoxesVector[0].y;
        point2.x = frameAct.segmLowerBoxesVector[1].x + frameAct.segmLowerBoxesVector[1].width/2;
        point2.y = frameAct.segmLowerBoxesVector[1].y;

//        cv::circle(frameAct.processFrame, point1, 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(frameAct.processFrame, point2, 2, CV_RGB(255, 0, 0), -1);

        result = distance(point1, point2);

        if (result < threshold) {
            frameAct.segmLowerBox.x = std::min(frameAct.segmLowerBoxesVector[0].x , frameAct.segmLowerBoxesVector[1].x);
            frameAct.segmLowerBox.y = std::min(point1.y, point2.y);
            frameAct.segmLowerBox.width = std::max(frameAct.segmLowerBoxesVector[0].x + frameAct.segmLowerBoxesVector[0].width,
                                                   frameAct.segmLowerBoxesVector[1].x + frameAct.segmLowerBoxesVector[1].width)
                                                 - frameAct.segmLowerBox.x;
            frameAct.segmLowerBox.height = std::max(point1.y + frameAct.segmLowerBoxesVector[0].height, point2.y +
                                           frameAct.segmLowerBoxesVector[1].height) - frameAct.segmLowerBox.y;
        }else{
            frameAct.segmLowerBox.x = frameAct.segmLowerBoxesVector[0].x;
            frameAct.segmLowerBox.y = frameAct.segmLowerBoxesVector[0].y;
            frameAct.segmLowerBox.width = frameAct.segmLowerBoxesVector[0].width;
            frameAct.segmLowerBox.height = frameAct.segmLowerBoxesVector[0].height;
        }
    }
}

void foot::getLowerBox(){

    frameAct.segmLowerBoxes.clear();
    frameAct.segmLowerBoxesVector.clear();

    getBlobsBoxes(frameAct.labelsFrame, frameAct.segmLowerBoxes);
    orderVectorBoxes(frameAct.segmLowerBoxes, frameAct.segmLowerBoxesVector);
    findLowerBox();

    cv::rectangle(frameAct.processFrame, frameAct.segmLowerBox, cyan, 2);

}

void foot::zoneDetection(geoproy GeoProy){

    cv::Point2f lowPointImage;

    lowPointImage.x = frameAct.segmLowerBox.x + frameAct.segmLowerBox.width/2; // NOLINT
    lowPointImage.y = frameAct.segmLowerBox.y + frameAct.segmLowerBox.height;

    frameAct.lowPointFloor = GeoProy.transformFloor2Image(lowPointImage, GeoProy.homographyInv); // NOLINT

    if (frameAct.lowPointFloor.y <= -50) {
        platformZone = 1;
    }else if (frameAct.lowPointFloor.y > -50 && frameAct.lowPointFloor.y <= 100){
        platformZone = 2;
    } else{
        platformZone = 3;
    }

}

/*
void foot::linearFunctionHeigth(){

    int h = frameAct.segmLowerBox.height;
    int newHeight;

    double hsizeMax;
    double hsizeMin;
    double percentMax;

    switch(platformZone) {
        case 1 :
            percentMax = 90.0;
            hsizeMax = 150.0;
            hsizeMin = 35.0;
            break;
        case 2 :
            percentMax = 115.0;
            hsizeMax = 150.0;
            hsizeMin = 30.0;
            break;
        default :
            percentMax = 70.0;
            hsizeMax = 150.0;
            hsizeMin = 40.0;
    }

    double slope = (percentMax/(hsizeMax-hsizeMin));
    double intercept = -((percentMax/(hsizeMax-hsizeMin))*hsizeMin);

    if(h > hsizeMin){
        frameAct.segmCutPercent = slope*h + intercept;
    }else{
        frameAct.segmCutPercent = 0;
    }

    newHeight = int (h * ((100 - frameAct.segmCutPercent)/100));
    frameAct.segmLowerBox.y += (h - newHeight);
    frameAct.segmLowerBox.height = newHeight;

}
*/

void foot::linearFunctionPosY(){

    int h = frameAct.segmLowerBox.height;
    int y = frameAct.lowPointFloor.y;
    int newHeight;
    double slope, intercept;

    //MinHeight
    int  hsizeMin = 30;

    //Zone1
    double percentMax1 = 30.0;
    double yMin1 = -300.0;
    double yMax1 = -50.0;

    //Zone2
    double percentMax2 = 80.0;
    double yMin2 = -50.0;
    double yMax2 = 100.0;

    //Zone3
    double percentMax3 = 75.0;
    double yMin3 = 100.0;
    double yMax3 = 300.0;

    switch(platformZone) {
        case 1 :
            slope = (percentMax1/(yMax1-yMin1));
            intercept = -((percentMax1/(yMax1-yMin1))*yMin1);
            break;
        case 2 :
            slope = (percentMax2-percentMax1)/(yMax2-yMin2);
            intercept = percentMax1-((percentMax2-percentMax1)/(yMax2-yMin2))*yMin2;
            break;
        default :
            percentMax2 = 60.0;
            slope = (percentMax3-percentMax2)/(yMax3-yMin3);
            intercept = percentMax2-((percentMax3-percentMax2)/(yMax3-yMin3))*yMin3;

    }

    if(h > hsizeMin){
        frameAct.segmCutPercent = slope*y + intercept;
    }else{
        frameAct.segmCutPercent = 0;
    }

    newHeight = int (h * ((100 - frameAct.segmCutPercent)/100));
    frameAct.segmLowerBox.y += (h - newHeight);
    frameAct.segmLowerBox.height = newHeight;

}


void foot::areasideFilter(std::map<int, cv::Rect> &bboxes){

    std::map<int, cv::Rect> areaFiltered, sideFilteredH, sideFilteredW;
    int thresholdArea = 160;
    int thresholdSideH = 8;
    int thresholdSideW = 5;

    for (int i = 0; i < bboxes.size() ; ++i) {
        if (bboxes[i].area() > thresholdArea)
            areaFiltered[i] = bboxes[i];
    }

    for (int j = 0; j < areaFiltered.size() ; ++j) {
        if (areaFiltered[j].height > thresholdSideH)
            sideFilteredH[j] = areaFiltered[j];
    }

    for (int k = 0; k < sideFilteredH.size(); ++k) {
        if (sideFilteredH[k].width > thresholdSideW)
            sideFilteredW[k] = sideFilteredH[k];
    }

    bboxes.clear();
    for (int l = 0; l < sideFilteredW.size(); ++l) {
        bboxes[l] = sideFilteredW[l];
    }
}

void foot::leftrightBoxes(){

    int Rigth = 2;
    int Left  = 1;
    int thresholdArea = 200;
    auto boxesSize = frameAct.footBoxes.size();


    switch(boxesSize) {
        case 1 :

            break;
        case 2 :
            cv::rectangle(frameAct.processFrame, frameAct.footBoxes[Left], green, 2);
            break;
        case 3 :
            cv::rectangle(frameAct.processFrame, frameAct.footBoxes[Left], red, 2);
            cv::rectangle(frameAct.processFrame, frameAct.footBoxes[Rigth], blue, 2);
            break;
        default :
            cv::rectangle(frameAct.processFrame, frameAct.footBoxes[Left], red, 2);
            cv::rectangle(frameAct.processFrame, frameAct.footBoxes[Rigth], blue, 2);
            break;
    }

//    cout << "boxesSize: " << boxesSize << endl;

}

void foot::getFeetBoxes(geoproy GeoProy){

    zoneDetection(std::move(GeoProy));
    //linearFunctionHeigth();
    linearFunctionPosY();

    Mat mask = Mat(frameAct.processFrame.size(), CV_8UC1, Scalar(0)); // NOLINT
    rectangle(mask, frameAct.segmLowerBox, Scalar(255), CV_FILLED);

    Mat fgROI = Mat::zeros(frameAct.processFrame.size(), CV_8U);
    // copia fg a fgROI donde mask es distinto de cero.
    frameAct.segmentedFrame.copyTo(fgROI, mask);

//    imshow("fgROI", fgROI);

    // aplica componentes conectados otra vez.
    cv::connectedComponents(fgROI, frameAct.labels2Frame, 8, CV_32S);

    frameAct.footBoxes.clear();
    frameAct.footBoxesVector.clear();

    getBlobsBoxes(frameAct.labels2Frame, frameAct.footBoxes);
    areasideFilter(frameAct.footBoxes);

    ////
    ////intentar pegar la pata
    ////

    orderVectorBoxes(frameAct.footBoxes, frameAct.footBoxesVector);


//    paintRectanglesVector(frameAct.footBoxesVector, green);
////    paintRectangles(frameAct.processFrame, frameAct.footBoxes, green);

    cv::Point center;
    for (int j = 0; j < frameAct.footBoxes.size() ; ++j) { //NOLINT
        center = (frameAct.footBoxes[j].br() + frameAct.footBoxes[j].tl())*0.5;
        cv::circle(frameAct.processFrame, center, 2, CV_RGB(0, 255, 0), -1);
    }

    leftrightBoxes();

}

//// Measure Foot ////
void foot::measureFoot(int pie){
    if(occlusion){
        if(pie == Right) {
            centerMeasured_R.x = frameAct.footBoxes[Right].x + frameAct.footBoxes[Right].width / 2;
            centerMeasured_R.y = frameAct.footBoxes[Right].y + frameAct.footBoxes[Right].height;
        }else{
            centerMeasured_L.x = frameAct.footBoxes[Right].x + frameAct.footBoxes[Right].width / 2; //(frameAct.footBoxes[Right].width*3) / 4;
            centerMeasured_L.y = frameAct.footBoxes[Right].y + frameAct.footBoxes[Right].height;
        }
    }else{
        if(pie == Right) {
            centerMeasured_R.x = frameAct.footBoxes[Right].x + frameAct.footBoxes[Right].width / 2;
            centerMeasured_R.y = frameAct.footBoxes[Right].y + frameAct.footBoxes[Right].height;
        }else{
            centerMeasured_L.x = frameAct.footBoxes[Left].x + frameAct.footBoxes[Left].width / 2;
            centerMeasured_L.y = frameAct.footBoxes[Left].y + frameAct.footBoxes[Left].height;
        }
    }
}




//// KALMAN FILTER ////


//// Kalman Initialization////
void foot::kalmanInit(int pie){

    cv::KalmanFilter kf;
    if(pie == Right) {
        kf = kf_R;
    }else{
        kf = kf_L;
    }

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 1e-2f;// 5.0f
    kf.processNoiseCov.at<float>(21) = 1e-2f;// 5.0f
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));

}

//// Kalman Prediction ////
void foot::kalmanPredict(int pie, int dT){

    cv::KalmanFilter *kf;
    cv::Mat *state;
    cv::Rect *predRect;
    cv::Point *centerKalman;

    if(pie == Right) {
        kf = &kf_R;
        state = &state_R;
        predRect = &predRect_R;
        centerKalman = &centerKalman_R;
    }else{
        kf = &kf_L;
        state = &state_L;
        predRect = &predRect_L;
        centerKalman = &centerKalman_L;
    }

    /////// Prediction ///////
    *state = (*kf).predict();

    ////// Predicted Rect Red //////
    (*predRect).width = static_cast<int>((*state).at<float>(4));
    (*predRect).height = static_cast<int>((*state).at<float>(5));
    (*predRect).x = static_cast<int>((*state).at<float>(0) - (*state).at<float>(4)/2);
    (*predRect).y = static_cast<int>((*state).at<float>(1) - (*state).at<float>(5));

    //// Predicted Point ////
    (*centerKalman).x = static_cast<int>((*state).at<float>(0));
    (*centerKalman).y = static_cast<int>((*state).at<float>(1));

    (*kf).transitionMatrix.at<float>(2) = dT;
    (*kf).transitionMatrix.at<float>(9) = dT;

}

//// Error Measure ////
void foot::measureError1Np(int pie){
    if (pie == Right){
        errorNpAct1_R = distance(centerKalman_R, centerMeasured_R);
    }else{
        errorNpAct1_L = distance(centerKalman_L, centerMeasured_L);
    }
}

//// Reset Kalman ////
void foot::kalmanResetStep(int pie){

    double error;
    bool reset;

    if (pie == Right){
            Reset_R = abs(errorNpAct1_R) > 2.2;//2
            reset = Reset_R;
    }else{
            Reset_L = abs(errorNpAct1_L) > 2.2;//2
            reset = Reset_L;
    }

    //if(!occlusion & !reset){
    if(!reset){
        if (pie == Right){
            error = (errorNpAct1_R); //+ errorNpAnt1_R)/2;
            if(abs(error) < 2.5) { //2.2
                step_R = true;
            }
            errorNpAnt1_R = errorNpAct1_R;
        }else{
            error = (errorNpAct1_L); //  + errorNpAnt1_L)/2;
            if(abs(error) < 2.5) { //2.2
                step_L = true;
            }
            errorNpAnt1_L = errorNpAct1_L;
        }

    }

}

//// Kalman Correction ////
void foot::kalmanUpdate(int pie){

    cv::KalmanFilter *kf;
    cv::Mat *state;
    cv::Mat *measure;
    bool *reset;

    if(pie == Right) {
        kf = &kf_R;
        state = &state_R;
        measure = &meas_R;
        reset = &Reset_R;
    }else{
        kf = &kf_L;
        state = &state_L;
        measure = &meas_L;
        reset = &Reset_L;
    }

    // Cuando no encuentra caja
    if (frameAct.footBoxes[1].width <= 0){
        notFoundCount++;
        if( notFoundCount >= 100 ){
            found = false;
        }else{
            (*kf).statePost.at<float>(0) = (*state).at<float>(0);
            (*kf).statePost.at<float>(1) = (*state).at<float>(1);//// + (*state).at<float>(5);
            (*kf).statePost.at<float>(2) = (*state).at<float>(2);
            (*kf).statePost.at<float>(3) = (*state).at<float>(3);
            (*kf).statePost.at<float>(4) = (*state).at<float>(4);
            (*kf).statePost.at<float>(5) = (*state).at<float>(5);
        }
    }else{

        if(pie == Right) {
            (*measure).at<float>(0) = frameAct.footBoxes[Right].x + float(frameAct.footBoxes[Right].width) / 2;
            (*measure).at<float>(1) = frameAct.footBoxes[Right].y + float(frameAct.footBoxes[Right].height);
            (*measure).at<float>(2) = (float) frameAct.footBoxes[Right].width;
            (*measure).at<float>(3) = (float) frameAct.footBoxes[Right].height;
        }else if(pie == Left){
            (*measure).at<float>(0) = frameAct.footBoxes[Left].x + float(frameAct.footBoxes[Left].width) / 2;
            (*measure).at<float>(1) = frameAct.footBoxes[Left].y + float(frameAct.footBoxes[Left].height);
            (*measure).at<float>(2) = (float) frameAct.footBoxes[Left].width;
            (*measure).at<float>(3) = (float) frameAct.footBoxes[Left].height;
        }

        if (*reset){ // First detection!
            // >>>> Initialization
            (*kf).errorCovPre.at<float>(0) = 1; // px
            (*kf).errorCovPre.at<float>(7) = 1; // px
            (*kf).errorCovPre.at<float>(14) = 1;
            (*kf).errorCovPre.at<float>(21) = 1;
            (*kf).errorCovPre.at<float>(28) = 1; // px
            (*kf).errorCovPre.at<float>(35) = 1; // px

            (*state).at<float>(0) = (*measure).at<float>(0);
            (*state).at<float>(1) = (*measure).at<float>(1);
            (*state).at<float>(2) = 0;
            (*state).at<float>(3) = 0;
            (*state).at<float>(4) = (*measure).at<float>(2);
            (*state).at<float>(5) = (*measure).at<float>(3);
            // <<<< Initialization

            (*kf).statePost.at<float>(0) = (*state).at<float>(0);
            (*kf).statePost.at<float>(1) = (*state).at<float>(1);
            (*kf).statePost.at<float>(2) = (*state).at<float>(2);
            (*kf).statePost.at<float>(3) = (*state).at<float>(3);
            (*kf).statePost.at<float>(4) = (*state).at<float>(4);
            (*kf).statePost.at<float>(5) = (*state).at<float>(5);

            *reset = false;

        }else{
            (*kf).correct((*measure)); // Kalman Correction
        }
        notFoundCount = 0;
    }

}



//// TEMPLATE NORMAL DETECTION ////


//// Generate Template ////
void foot::generateTemplateNp(){

    int xr, yr, wr, hr;
    int xl, yl, wl, hl;

    if (frameAct.footBoxes[Right].width > 0 && frameAct.footBoxes[Left].width > 0 &&
            frameAct.footBoxes[Right].height > 0 && frameAct.footBoxes[Left].height > 0 ){

        xr = frameAct.footBoxes[Right].x - offsetR; yr = frameAct.footBoxes[Right].y - offsetR;
        wr = frameAct.footBoxes[Right].width + 2*offsetR; hr = frameAct.footBoxes[Right].height + 2*offsetR;

        xl = frameAct.footBoxes[Left].x - offsetL; yl = frameAct.footBoxes[Left].y - offsetL;
        wl = frameAct.footBoxes[Left].width + 2*offsetL; hl = frameAct.footBoxes[Left].height + 2*offsetL;

        if (xr > 0 && xl > 0 && yr > 0 && yl > 0){

            Rect roifootR(xr, yr, wr, hr);
            Rect roifootL(xl, yl, wl, hl);

            frameAct.tempBoxes[Right] = roifootR;
            frameAct.tempBoxes[Left]  = roifootL;

            frameAct.templateFrameR = frameAct.processFrame(roifootR);
            frameAct.tempmaskFrameR = frameAct.segmentedFrame(roifootR);

            frameAct.templateFrameL = frameAct.processFrame(roifootL);
            frameAct.tempmaskFrameL = frameAct.segmentedFrame(roifootL);

        }
    }

}


//// OCCLUSION ////


//// Matching Score Partial Occlusion
//// gets the matching score for Right and Left foot.
void foot::matchingScorePocc(){

    int offset_oc = 10;
    int xoc = frameAct.footBoxes[1].x - offset_oc,  yoc = frameAct.footBoxes[1].y - offset_oc;
    int woc = frameAct.footBoxes[1].width + 2*offset_oc, hoc = frameAct.footBoxes[1].height + 2*offset_oc;

    cv::Rect roioc(xoc, yoc, woc, hoc);

    occlusionCorner.x = xoc;
    occlusionCorner.y = yoc;

    frameAct.occlusionFrame = frameAct.processFrame(roioc);
    frameAct.occlumaskFrame = frameAct.segmentedFrame(roioc);

    cv::Mat matchScoreR, matchScoreL, matchScoreShowR, matchScoreShowL;

    matchTemplate(frameAct.occlusionFrame, frameAnt.templateFrameR, matchScoreR, CV_TM_SQDIFF_NORMED);
    matchTemplate(frameAct.occlusionFrame, frameAnt.templateFrameL, matchScoreL, CV_TM_SQDIFF_NORMED);

    matchScoreR = 1 - matchScoreR;
    matchScoreL = 1 - matchScoreL;
    normalize(matchScoreR, matchScoreR, 255, 0, NORM_MINMAX);
    normalize(matchScoreL, matchScoreL, 255, 0, NORM_MINMAX);
    matchScoreR.convertTo(matchScoreR, CV_8UC1);     // NOLINT
    matchScoreL.convertTo(matchScoreL, CV_8UC1);     // NOLINT
    matchScoreR.convertTo(matchScoreShowR, CV_8UC1); // NOLINT
    matchScoreL.convertTo(matchScoreShowL, CV_8UC1); // NOLINT
    applyColorMap(matchScoreShowR, matchScoreShowR, COLORMAP_JET);
    applyColorMap(matchScoreShowL, matchScoreShowL, COLORMAP_JET);

    frameAct.matchScoreR = matchScoreR;
    frameAct.matchScoreShowR = matchScoreShowR;
    frameAct.matchScoreL = matchScoreL;
    frameAct.matchScoreShowL = matchScoreShowL;

}

//// Found Local Maximum ////
void foot::occlusionType(){

    cv::Mat segmentationR, segmentationL;
    cv::Mat erodedR, erodedL;
    cv::Mat statusR, statusL;
    cv::Mat componentsR, componentsL;

    //// Segmentation ////
    threshold(frameAct.matchScoreR, segmentationR, 190, 255, THRESH_BINARY);
    threshold(frameAct.matchScoreL, segmentationL, 190, 255, THRESH_BINARY);
    //// Eroded ////
    erode(segmentationR, erodedR, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    erode(segmentationL, erodedL, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
    //// Connected Component ////
    connectedComponentsWithStats(erodedR, componentsR, statusR, centroidsR, 4, CV_32S);
    connectedComponentsWithStats(erodedL, componentsL, statusL, centroidsL, 4, CV_32S);

    totalOccR = centroidsR.rows <= 2;
    totalOccL = centroidsL.rows <= 2;

}

//// Max Candidates Points Vector ////
//// Points Corrected for Kalman Compare ////
void foot::maxCandidatesPocc(){

    cv::Point centro_masa;

    for (int i = 0; i < centroidsR.rows - 1; ++i) {
        centro_masa.x = int(centroidsR.at<double>(i+1,0));
        centro_masa.y = int(centroidsR.at<double>(i+1,1));
        maxLocR.push_back(centro_masa);
    }
    for (int i = 0; i < centroidsL.rows - 1; ++i) {
        centro_masa.x = int(centroidsL.at<double>(i+1,0));
        centro_masa.y = int(centroidsL.at<double>(i+1,1));
        maxLocL.push_back(centro_masa);
    }

}

//// Measure Distance Kalman vs Matching ////
void foot::matchingSelectPocc(){

    double distR, distL, distMaxR = 1000, distMaxL = 1000;
    cv::Point bestPointR, bestPointL;

    for (const auto &i : maxLocR) {
        bestPointR = i;
        bestPointR.y = bestPointR.y + frameAnt.templateFrameR.rows + occlusionCorner.y;
        bestPointR.x = bestPointR.x + (frameAnt.templateFrameR.cols/2) + occlusionCorner.x;
        distR = distance(centerKalman_R, bestPointR);
        if (distR < distMaxR){
            distMaxR = distR;
            maxlocSelectedR = i;
        }
    }
    for (const auto &i : maxLocL) {
        bestPointL = i;
        bestPointL.y = bestPointL.y + frameAnt.templateFrameL.rows + occlusionCorner.y;
        bestPointL.x = bestPointL.x + (frameAnt.templateFrameL.cols/2) + occlusionCorner.x;
        distL = distance(centerKalman_L, bestPointL);
        if (distL < distMaxL){
            distMaxL = distL;
            maxlocSelectedL = i;
        }
    }
}

//// Update FootBoxes in Partial Occlusion ////
//// considera proyectar tomando en cuenta el punto mas bajo en template mask ////
void foot::proyectBoxes() {

    frameAct.footBoxes[Right]   = predRect_R;
    frameAct.footBoxes[Right].x = maxlocSelectedR.x + occlusionCorner.x - offsetR/2;
    frameAct.footBoxes[Right].y = maxlocSelectedR.y + occlusionCorner.y - offsetR/2;

    frameAct.footBoxes[Left]   = predRect_L;
    frameAct.footBoxes[Left].x = maxlocSelectedL.x + occlusionCorner.x - offsetL/2;
    frameAct.footBoxes[Left].y = maxlocSelectedL.y + occlusionCorner.y - offsetL/2;

}




//// OBJETIVE MATCHING ////


//// Ask which objetive is near of step given ////


//// MEDIR DISTANCIA EN EL PISO!!!!
//// PARA PODER COMPARAR EN CENTIMETROS!!!!

void foot::askObjetives(geoproy GeoProy){

    int totalObjetives = 9;
    double objetiveThreshold = 30;
    double resultDistance;

    cv::Point stepPoint;

    if (step_L){
        stepPoint = geoproy::transformFloor2Image(centerKalman_L, GeoProy.homographyInv);
        cout << "StepPoint L: " << stepPoint << endl;
        for (int i = 0; i < totalObjetives; ++i) {
            resultDistance = distance(stepPoint, GeoProy.calibPointsFloor[i]);
            if (resultDistance < objetiveThreshold){
                cout << "Objetivo Reach L: " << GeoProy.calibPointsFloor[i] << endl;
                break;
            }

        }

    }
    if (step_R){
        stepPoint = geoproy::transformFloor2Image(centerKalman_R, GeoProy.homographyInv);
        cout << "StepPoint R: " << stepPoint << endl;
        for (int i = 0; i < totalObjetives; ++i) {
            resultDistance = distance(stepPoint, GeoProy.calibPointsFloor[i]);
            if (resultDistance < objetiveThreshold){
                cout << "Objetivo Reach R: " << GeoProy.calibPointsFloor[i] << "\n" << endl;
                break;
            }

        }

    }


}

























//// DRAW RESULTS ////


//// Draw Results to Image ////
void foot::drawingResults() {

    //// Foots Rectangles ////
    paintRectangles(frameAct.resultFrame, frameAct.footBoxes, green);

    //// Measured Centers ////
//    cv::circle(frameAct.resultFrame, centerMeasured_R, 3, green, -1);
//    cv::circle(frameAct.resultFrame, centerMeasured_L, 3, green, -1);

//    if(!occlusion) {



        //// Kalman Prediction ////
//        cv::rectangle(frameAct.resultFrame, predRect_R, CV_RGB(255, 0, 0), 2);
//        cv::rectangle(frameAct.resultFrame, predRect_L, CV_RGB(255, 0, 0), 2);
//        cv::circle(frameAct.resultFrame, centerKalman_R, 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(frameAct.resultFrame, centerKalman_L, 2, CV_RGB(255, 0, 0), -1);

        //// Template Boxes Generated in Normal Detection ////
        //paintRectangles(frameAct.resultFrame, frameAct.tempBoxes, blueviolet);


//    }
//    //// Matchscore Partial Occlusion ////
//    }else{
//
//        //// Kalman Prediction ////
//        cv::rectangle(frameAct.resultFrame, predRect_R, CV_RGB(255, 0, 0), 2);
//        cv::rectangle(frameAct.resultFrame, predRect_L, CV_RGB(255, 0, 0), 2);
//        cv::circle(frameAct.resultFrame, centerKalman_R, 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(frameAct.resultFrame, centerKalman_L, 2, CV_RGB(255, 0, 0), -1);
//
//        //// Predicted Boxes ////
//        paintRectangles(frameAct.resultFrame, frameAct.footBoxes, cyan);
//
//
//        namedWindow("Occlusion", WINDOW_AUTOSIZE);
//        namedWindow("TempR", WINDOW_AUTOSIZE);
//        namedWindow("TempL", WINDOW_AUTOSIZE);
//        namedWindow("MatchR", WINDOW_AUTOSIZE);
//        namedWindow("MatchL", WINDOW_AUTOSIZE);
//
//        Size sizeoccBox(frameAct.occlusionFrame.cols*10, frameAct.occlusionFrame.rows*10);
//        Size sizetempBoxR(frameAnt.templateFrameR.cols*10, frameAnt.templateFrameR.rows*10);
//        Size sizetempBoxL(frameAnt.templateFrameL.cols*10, frameAnt.templateFrameL.rows*10);
//        Size sizematchScoreR(frameAct.matchScoreShowR.cols*10, frameAct.matchScoreShowR.rows*10);
//        Size sizematchScoreL(frameAct.matchScoreShowL.cols*10, frameAct.matchScoreShowL.rows*10);
//
//        //// Paint Local Max Points ////
//        for (const auto &i : maxLocR) {
//            circle(frameAct.matchScoreShowR, i, 1, CV_RGB(0,0,255), -1);
//        }
//        for (const auto &i : maxLocL) {
//            circle(frameAct.matchScoreShowL, i, 1, CV_RGB(0,0,255), -1);
//        }
//
//        circle(frameAct.matchScoreShowR, maxlocSelectedR, 1, blueviolet, -1);
//        circle(frameAct.matchScoreShowL, maxlocSelectedL, 1, blueviolet, -1);
//
//        resize(frameAct.occlusionFrame, frameAct.occlusionFrame, sizeoccBox);
//        resize(frameAnt.templateFrameR, frameAnt.templateFrameR, sizetempBoxR);
//        resize(frameAnt.templateFrameL, frameAnt.templateFrameL, sizetempBoxL);
//        resize(frameAnt.tempmaskFrameR, frameAnt.tempmaskFrameR, sizetempBoxR);
//        resize(frameAnt.tempmaskFrameL, frameAnt.tempmaskFrameL, sizetempBoxL);
//        resize(frameAct.matchScoreShowR, frameAct.matchScoreShowR, sizematchScoreR);
//        resize(frameAct.matchScoreShowL, frameAct.matchScoreShowL, sizematchScoreL);
//
//        imshow("Occlusion", frameAct.occlusionFrame);
//        imshow("TempR", frameAnt.tempmaskFrameR);
//        imshow("TempL", frameAnt.tempmaskFrameL);
//        imshow("MatchR", frameAct.matchScoreShowR);
//        imshow("MatchL", frameAct.matchScoreShowL);
//
//
//    }

    //// Step Detected ////
    if (step_R) {
        cv::rectangle(frameAct.resultFrame, frameAct.footBoxes[Right], CV_RGB(0, 0, 255), 2);
        cv::circle(frameAct.resultFrame, centerMeasured_R, 2, orange, -1);
    }
    if (step_L) {
        cv::rectangle(frameAct.resultFrame, frameAct.footBoxes[Left], CV_RGB(0, 0, 255), 2);
        cv::circle(frameAct.resultFrame, centerMeasured_L, 2, ivory, -1);
    }

}

//// Clear Variables ////
void foot::clearVariables(){

    maxLocR.clear();
    maxLocL.clear();

}