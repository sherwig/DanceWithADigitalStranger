#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include <vector>


//#define HOST "localhost"
//#define HOST "10.0.0.144" //pc
//#define HOST "10.0.0.213"

/// send port
//#define PORT 14747
//#define PORT2 9008

#define NUM_MSG_STRINGS 20

class ofApp : public ofBaseApp {
public:
    void setup();
    void update();
    void draw();
    void exit();
    float accel2(float speedVal);
    void FileWrite();
    void drawShapes();
    
    void drawInstructions();
    void drawPointCloud();
    
    void keyPressed(int key);
    
    void calculateText();
    
    ofxKinect kinect;
    ofxCv::ContourFinder contourFinder;
    
    ofImage colorImg;
    ofImage grayImage;         // grayscale depth image
    ofImage grayThreshNear;    // the near thresholded image
    ofImage grayThreshFar;     // the far thresholded image
    ofImage grayPreprocImage;  // grayscale pre-processed image
    
    //Contour Finding Kinect
    int nearThreshold;
    int farThreshold;
    int angle;
    
    //Booleans
    bool drawStuff;
    bool clearStuff;
    bool bDrawPointCloud;
    bool rotating;
    bool drawBackground;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    bool showLabels;
    
    //Randomization of color
    ofVec2f p;
    ofColor col;
    ofColor col2;
    ofColor col3;
    ofColor col4;
    ofColor col5;
    ofColor colCos;
    ofVec3f p2;
    ofMatrix4x4 m;
    
    
    ofPolyline currentPolyline;
    vector <ofPolyline> polylines;
    
    int minDistance;
    
    std::vector <ofVec3f> vec;
    std::vector <ofVec2f> vecCopy;
    vector <ofVec3f> offsets;
    
    ofVec3f vert2;
    
    ofMesh mesh;
    ofMesh meshCopy;
    
    ofMesh mesh2;
    ofMesh meshCopy2;
    //vector<ofVec3f> offsets;
    
    bool orbiting;
    float startOrbitTime;
    vector<float> distances;
    vector<float> angles;
    
    vector<float> distances2;
    vector<float> angles2;
    
    ofVec3f meshCentroid;
    ofVec3f meshCentroid2;
    
    vector<ofVec3f> storage;
    
    ofImage image;
    
    float speedVal;
    
    ofCamera camera;
    
    float cameraX;
    float cameraY;
    float cameraZ;
    
    int nextIndexToWrite;
    int nextIndexToWrite2;
    int nextIndexToWrite3;
    
    vector<vector<ofVec3f>> mat;
    
    int R=1;
    int C=10;
    
    ofJson js;
    ofJson stroke;
    ofTrueTypeFont ttf;
    ofPath path;
    std::string text;
    
    ofxOscSender sender;
    ofxOscReceiver receiver; 
    
    float pointSize;
    float sinConnectionDistance;
    float scaleMesh;
    float meshMode;
    
    float tempArea;
    int tempSize;
    ofVec3f tempReciever;
    
    vector<ofVec3f> sendVec;
    
    float tempScale;
    
    
    int thresh1;
    int thresh2;
    int thresh3;
    int thresh4;
    int thresh5;
    
    
    int colThresh1;
    int colThresh2;
    int colThresh3;
    int colThresh4;
    int colThresh5;
    
    
    int mainThresh1;
    int mainThresh2;
    
    
    
};
