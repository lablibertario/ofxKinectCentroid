#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    void guiSetup();
    void processContour();
    void updateAngle(int& angle);

    ofxKinect kinect;
    ofxCvGrayscaleImage ImagenDepth;
    ofxCvGrayscaleImage ImagenGrises;
    ofxCvGrayscaleImage GrisesUmbralCerca;
    ofxCvGrayscaleImage GrisesUmbralLejos;
    ofxCv::ContourFinder contourFinder;

    ofxPanel gui;
    ofParameterGroup parametersKinect;
    ofParameter<int> lejosThreshold;
    ofParameter<int> cercaThreshold;
    ofParameter<int> angle;
    ofParameter<int> numMaxBlobs;
    ofParameter<int> TamMinBlob;
    ofParameter<int> TamMaxBlob;
    


};

