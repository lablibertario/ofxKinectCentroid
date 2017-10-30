#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    kinect.setRegistration(true);
    kinect.init(true);
    
    ImagenDepth.allocate(kinect.width, kinect.height);
    ImagenGrises.allocate(kinect.width, kinect.height);
    GrisesUmbralCerca.allocate(kinect.width, kinect.height);
    GrisesUmbralLejos.allocate(kinect.width, kinect.height);
    
    cercaThreshold = 230;
    lejosThreshold = 70;
    
    contourFinder.setMinAreaRadius(10);
    contourFinder.setMaxAreaRadius(150);

    
    ofSetFrameRate(60);
    
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    
    guiSetup();
    

}

void ofApp::guiSetup(){
    
    gui.setup("Settings", "settings.xml");
        parametersKinect.setName("Controles");
        parametersKinect.add(lejosThreshold.set("Umbral Lejos", 0,0, 255 ));
        parametersKinect.add(cercaThreshold.set("Umbral Cerca", 0,0, 255 ));
        parametersKinect.add(numMaxBlobs.set("Num Max Blobs",10,0,1000));
        parametersKinect.add(TamMaxBlob.set("Tam Max Blob",0,0,500));
        parametersKinect.add(TamMinBlob.set("Tam Min Blob",0,0,500));
        parametersKinect.add(angle.set("Angulo Kinect",0,-30,30));
    
        angle.addListener(this,&ofApp::updateAngle);

        gui.add(parametersKinect);
    
        gui.loadFromFile("settings.xml");
        gui.setPosition(20,340);
}

void ofApp::update() {
    ofBackground(200, 200, 200);
    kinect.update();
        if(kinect.isFrameNew()) {
        
        ImagenDepth.setFromPixels(kinect.getDepthPixels());
        ImagenDepth.mirror(0,1);
        ImagenGrises.setFromPixels(kinect.getDepthPixels());
        ImagenGrises.mirror(0,1);
        GrisesUmbralCerca = ImagenGrises;
        GrisesUmbralLejos = ImagenGrises;
        GrisesUmbralCerca.threshold(cercaThreshold, true);
        GrisesUmbralLejos.threshold(lejosThreshold);
        cvAnd(GrisesUmbralCerca.getCvImage(), GrisesUmbralLejos.getCvImage(), ImagenGrises.getCvImage(), NULL);
    
        ImagenGrises.flagImageChanged();

        
        contourFinder.setMinAreaRadius(TamMinBlob);
        contourFinder.setMaxAreaRadius(TamMaxBlob);
        contourFinder.findContours(ImagenGrises);
        
    }

}

void ofApp::draw() {
    
    ofSetColor(255, 255, 255);
    ofPushMatrix();
        ofScale(0.66,0.66);
        ImagenDepth.draw(0, 0);
        ImagenGrises.draw(640, 0);
        ofTranslate(640,0);
        contourFinder.draw();
    ofPopMatrix();
    
    ofSetColor(255, 255, 255);
    processContour();
    gui.draw();

    
}
    
void ofApp::processContour() {
    
   // ofxOscBundle bundle;
    RectTracker& tracker = contourFinder.getTracker();
    int count = 0;
    
    vector<float> objectsDepth;
    
    auto depthPixels = ImagenGrises.getPixels();
    
    ofPushMatrix();
        ofScale(0.66,0.66);
        ofTranslate(640,0);
        for(int i=0; i < contourFinder.size(); i++){
            
            unsigned int label = contourFinder.getLabel(i);
            
            if(tracker.existsPrevious(label)) {
            
                // rodeamos con ellipse el mejor contorno
                ofSetColor(magentaPrint);
                RotatedRect ellipse = contourFinder.getFitEllipse(i);
                ofPushMatrix();
        
                    ofVec2f ellipseCenter = toOf(ellipse.center);
                    ofVec2f ellipseSize = toOf(ellipse.size);
                    ofTranslate(ellipseCenter.x, ellipseCenter.y);
                    ofRotate(ellipse.angle);
                    ofNoFill();
                    ofSetColor(255,0,0);
                    ofDrawEllipse(0, 0, ellipseSize.x, ellipseSize.y);
                
                ofPopMatrix();
                
                ofVec2f center = toOf(contourFinder.getCenter(i));

                int px0 = ellipseCenter.x - ellipseSize.x/2;
                int px1 = ellipseCenter.x + ellipseSize.x/2;

                px0 = ofClamp(px0, 0, kinect.width);
                px1 = ofClamp(px1, 0, kinect.width);

                float py0 = ellipseCenter.y - ellipseSize.y/2;
                float py1 = ellipseCenter.y + ellipseSize.y/2;
            
                py0 = ofClamp(py0, 0, kinect.height);
                py1 = ofClamp(py1, 0, kinect.height);

                
                float distanceValue = 0;
                int countAve = 0;
                for(int pointX = px0; pointX < px1; pointX++){
                    for(int pointY = py0; pointY < py1; pointY++){
                        distanceValue +=depthPixels[pointY * ImagenGrises.width + pointX];
                        countAve++;
                    }
                }
            
                distanceValue = 255-(distanceValue/countAve);
                
                objectsDepth.push_back(distanceValue);

                ofSetColor(0,0,255);
                ofFill();
                ofDrawCircle(center, distanceValue/10);
                
                // convex hull del contorno
                ofSetColor(yellowPrint);
                ofPolyline convexHull = toOf(contourFinder.getConvexHull(i));
                convexHull.draw();
                
                // defectos del convex hull
                vector<cv::Vec4i> defects = contourFinder.getConvexityDefects(i);
                for(int j = 0; j < defects.size(); j++) {
                    ofDrawLine(defects[j][0], defects[j][1], defects[j][2], defects[j][3]);
                }
                
                kinect.getDepthPixels();
                count++;
            }
        }
    
    ofPopMatrix();
       
}
    

void ofApp::exit() {
    kinect.close();
}

void ofApp::updateAngle(int &angle){
    
    kinect.setCameraTiltAngle(angle);

}


