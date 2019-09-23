#include "ofApp.h"
#include <vector>

using namespace ofxCv;
using namespace cv;


void ofApp::setup() {
    
    ofSetLogLevel(OF_LOG_VERBOSE);

    kinect.init();
    
    kinect.open();
    
    p.set( ofGetWidth() * 0.5f, ofGetHeight() * 0.5f );
    
    drawStuff=false;
    // Print the intrinsic IR sensor values
    if (kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    // Thresholds for kinect.
    nearThreshold = 255; //230;
    farThreshold = 220; //70;
    
    // Zero the tilt on startup
    //angle = 0;
    // kinect.setCameraTiltAngle(angle);
    
    // Start from the front
    bDrawPointCloud = false;
    clearStuff=false;
    rotating=false;
    
    
    col.r=0;
    col.g=255;
    col.b=0;
    
    float accelh=1;
    
    // Allocate images
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshNear.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshFar.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayPreprocImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

    contourFinder.setFindHoles(false);
    
    ofSetFrameRate(30);
    showLabels = true;
    
    contourFinder.setMinAreaRadius(1);
    contourFinder.setMaxAreaRadius(300);
    contourFinder.setThreshold(205);
    // wait for half a second before forgetting something
    contourFinder.getTracker().setPersistence(100);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    
    minDistance=10;
    
    //Just using a easy cam right now
//    camera.setFov(60);
//    cameraX=0;
//    cameraY=-25;
//    cameraZ=0;
    
    ofSetBackgroundColor(255);
    ttf.load("mono.ttf", 8);
    path.setStrokeColor(0);
    path.setFilled(false);
    path.setStrokeWidth(1);
    
    ofFile file("drawing copy.json");
    if(file.exists()){
        file >> js;
        for(auto & stroke: js){
            if(!stroke.empty()){
                path.moveTo(stroke[0]["x"],stroke[0]["y"],stroke[0]["z"]);
                for(auto & p: stroke){
                    path.lineTo(p["x"], p["y"], p["z"]);
                }
            }
        }
        
       // calculateText();
    }
    
    
}
//ofSetFrameRate(60);



//--------------------------------------------------------------
void ofApp::update()
{
    float time= ofGetElapsedTimef();
    for(int i=0; i<1000; i++)
    {
        //ofSetColor(127+127*sin(i*.01+time),127+127*sin(i*.012+time),127+127*sin(i*.014+time));
        //pathFromContour.setFillColor(127+127*sin(i*.01+time),127+127*sin(i*.012+time),127+127*sin(i*.014+time));
        
        //Color for the person on offset sin curves.
                col.r = 127+127*2*sin(i*.01+time);
                col.g = 127+127*sin(i*.012+time);
                col.b = 127+127*sin(i*.014+time);
        
        
                col2.r = 127+127*2*sin(i*.017+time);
                col2.g = 127+127*sin(i*.018+time);
                col2.b = 127+127*sin(i*.012+time);
        
                col3.r = 127+127*2*sin(i*.018+time);
                col3.g = 127+127*sin(i*.20+time);
                col3.b = 127+127*sin(i*.020+time);
        
                col4.r = 127+127*2*sin(i*.08+time);
                col4.g = 127+127*sin(i*.1+time);
                col4.b = 127+127*sin(i*.01+time);
        
                col5.r = 127+127*2*sin(i*.06+time);
                col5.g = 127+127*sin(i*.09+time);
                col5.b = 127+127*sin(i*.013+time);
        
        
        
    }
    
    ofBackground(10, 10, 10);
    
    kinect.update();
    
    // There is a new frame and we are connected
    //All thresholding code for the contour.
    if (kinect.isFrameNew()) {

        grayImage.setFromPixels(kinect.getDepthPixels());
        
        
        // Threshold image
        threshold(grayImage, grayThreshNear, nearThreshold, true);
        threshold(grayImage, grayThreshFar, farThreshold);
        
        // Convert to CV to perform AND operation
        Mat grayThreshNearMat = toCv(grayThreshNear);
        Mat grayThreshFarMat = toCv(grayThreshFar);
        Mat grayImageMat = toCv(grayImage);
        
        // cvAnd to get the pixels which are a union of the two thresholds
        bitwise_and(grayThreshNearMat, grayThreshFarMat, grayImageMat);
        
        // Save pre-processed image for drawing it
        grayPreprocImage = grayImage;
        
        // Process image
        dilate(grayImage);
        dilate(grayImage);
        
        erode(grayImage);
        
        // Mark image as changed
        grayImage.update();

        contourFinder.findContours(grayImage);
    }
    
    
    
    
    
    

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
    RectTracker& tracker = contourFinder.getTracker();
    
    easyCam.begin();
    //camera.begin();
    
    //Drawing the person.
   
    //drawPointCloud();
   
    drawShapes();
    
    //    ofColor centerColor = ofColor(85, 78, 68);
    //    ofColor edgeColor(0, 0, 0);
    //    ofBackgroundGradient(centerColor, edgeColor, OF_GRADIENT_CIRCULAR);
    //if(drawBackground)
    //{
    //
    //
    
    //contourFinder.draw();
    
    //}

//            if (vecCopy.empty())
//            {
//                vecCopy.clear();
//                vecCopy.push_back(ofVec3f(0,0,0));
//                cout<<vecCopy.size()<<endl;
//            }
//
//            else
//            {
//                for (int j=0; j<Numb_delete2; j++)
//                {
//                    vecCopy.erase(vecCopy.begin());
//                }
//            }

               // center = toOf(contourFinder.getCenter(i));
        
        
        //Here is the problem child. Right now I am just trying to transfer over the vector values from the original vec to the vecCopy that I will use to draw at a later time. I want to focus on this firtly and then worry about the time aspect later.
        //If you comment all of this out the sketch runs fine jsut like how I showed you after class.
        
//        for(int j=vecCopy.size()-1; j<numb_shapes; j++)
//                {
//                    //Basically adapting the code from above for filling the vector.
//
//                    if (vecCopy.size()<vec.size())//Not sure if this will work.
//                    {
//                        vecCopy.push_back(vec[j]);//push back the vector values into the vec copy.
//                        cout<<vecCopy.size()<<endl;//see if it is getting filled.
//                    }
//
//                    else//so if VecCopy is as large as the vec then it will delete off the front but not sure if this will work either.
//                    {
//                        for (int i=0; i<Numb_delete2; i++)
//                        {
//                            vecCopy.erase(vecCopy.begin());
//
//                        }
//                    }
//
//                    //Set size of vector array have a current index instead of vector overwriting existing value
//                    //need to iterate background until index -- go through full index
    
//                    if (vecCopy.size()>4)//Go through and draw the copied vector
//                    {
//                        for (int r=0; r<vecCopy.size()-4; r++)
//                        {
//                            //using the same styling from before.
//                            ofNoFill();
//                            int k=vec.size()-1;
//                            float value=255.0/max(vec[k].x,vec[k].y);
//                            //ofSetColor(uint8(value*vec[k].x), uint8(value*vec[k].y),uint8(value*vec[k].z)); //Rainbow
//                            //ofSetColor(uint8(value*vec[k].x), ofRandom(0,100),0); //RED
//                            ofSetColor(ofRandom(0,255), uint8(value*vec[k].y),uint8(value*vec[k].x)); //Blue
//
//                            //Translate it up 300 so I can see the difference.
//                            ofTranslate(0,300,0);
//                            //Draw the vec copy the same way.
//                            ofDrawTriangle(vecCopy[r].x,vecCopy[r].y,200,vecCopy[(r+1)].x,vecCopy[(r+1)].y,300, vecCopy[(r+2)].x,vecCopy[(r+2)].y,100);
//
//                        }
//
//                   }
//                }
//

     easyCam.end();
    
}

void ofApp::calculateText(){
    auto total = js;
    if(!stroke.empty()){
        total.push_back(stroke);
    }
    size_t numlines = ofGetHeight() / ttf.getLineHeight();
    auto lines = ofSplitString(total.dump(2), "\n");
    std::vector<std::string> screenlines(lines.end() - std::min(lines.size(), numlines), lines.end());
    text = ofJoinString(screenlines, "\n");
}

void ofApp::drawShapes()
{
     int counter=0;
    //boolean that is mapped to the b button to draw the triangles.
    if (drawStuff)
    {
        //setting number of shapes that I am going to fill my array with and the number that I will be deleting from the front of the vector.
//        int numb_shapes=500;
//        int Numb_delete=10;
//
//        if (vec.empty())
//        {
//            vec.clear();
//            vec.push_back(ofVec3f(0,0,0));
//            counter=0;
//        }
//        else
//        {
//              counter++;
////            while(counter<=numb_shapes)
////            {
//                counter++;
//                if(counter==numb_shapes)
//                {
//                    counter=0;
//                }
//                ofPoint center = toOf(contourFinder.getCenter(counter));
//                //go through and push back to the vector based on the center of the contour. This only works well really with one person in the frame, which I might need to go through and update later on.
//                    ofVec2f temp_pos;
//                    temp_pos.x=center.x;
//                    temp_pos.y=center.y;
//                    vec.push_back(temp_pos);
//             }
        //}
        
        
       // glm::vec2 position = getPosition();
       
//
//        for (int i=0; i<10; i++)
//        {
//           for (int j=0; j<1;j++)
//           {
        
          //  ofTranslate(0,200);
            ofPoint center = toOf(contourFinder.getCenter(counter));
            //go through and push back to the vector based on the center of the contour. This only works well really with one person in the frame, which I might need to go through and update later on.
                ofVec2f temp_pos;
                temp_pos.x=center.x;
                temp_pos.y=center.y;
                //vec.push_back(temp_pos);
            
            const std::size_t MAX_BUFFER_SIZE = 100;
            
        
        
            // This section fills the buffer.
            if (vec.size() < MAX_BUFFER_SIZE)
            {
                vec.push_back(temp_pos);
              
                nextIndexToWrite = vec.size();
            }
        
            else // This section reuses positions once it is full.
            {
                vec[nextIndexToWrite] = temp_pos;
            }
            
            // This section figures out the next write index and
            nextIndexToWrite = (nextIndexToWrite + 1) % MAX_BUFFER_SIZE;
       
        //fill 10 vectors you are trying to fill 10 matrix spots with 1 vector that is filled currently
             
            //}

                 //mat[i].push_back(vec);
//
       //}

        //cout<<vec.size()<<endl;
        //cout<<counter<<endl;
        
        const std::size_t MAX_BUFFER_SIZE2 = 100;

        // This section fills the buffer.
        if (vecCopy.size() < MAX_BUFFER_SIZE2)
        {
            vecCopy.push_back(temp_pos);
            nextIndexToWrite2 = vecCopy.size();
        }
        else // This section reuses positions once it is full.
        {
            vecCopy[nextIndexToWrite2] = temp_pos;
        }

        // This section figures out the next write index and
        nextIndexToWrite2 = (nextIndexToWrite2 + 1) % MAX_BUFFER_SIZE2;
        
//        int numb_shapes2=500;
//        int Numb_delete2=10;
        
        //translating and scaling the triangles to be aroung the person as well as being large.
        ofTranslate(-2200, 2500, 600);
        ofScale(8, -8, -8);
        if (vec.size()>4) //if there is something in the vector.
        {
            for (int r=0; r<vec.size()-4; r++)
            {
                //ofDrawCircle(vec[r].x-400,vec[r].y-400,10);
                
                ofNoFill();
                int k=vec.size()-1;
                float value=255.0/max(vec[k].x,vec[k].y); // Code to find the max of x and y points and map it to 0-255 for some color.
                //ofSetColor(uint8(value*vec[k].x), uint8(value*vec[k].y),uint8(value*vec[k].z)); //Rainbow
                //ofSetColor(uint8(value*vec[k].x), ofRandom(0,100),0); //RED
                
                //randomize the first value and then use the other two values to set the color.
                //Am not doing it with z value because I am having a hard time figuring out the z value as in tracking the depth fo the person and using that for the current z value. This is an interesting thing that I want to figure out.
                ofSetColor(ofRandom(0,255), uint8(value*vec[k].y),uint8(value*vec[k].x)); //Blue
                
                //ofTranslate(-100, -100);
                //ofDrawSphere(vec[r].x-500,vec[r].y-500,ofRandom(1000),10);
                //ofDrawLine(vec[r].x,vec[r].y,vec[(r+1)].x,vec[(r+1)].y);
                // ofDrawTriangle(vec[r].x,vec[r].y,ofRandom(200),vec[(r+1)].x,vec[(r+1)].y,ofRandom(100), vec[(r+2)].x,vec[(r+2)].y,ofRandom(-400));
                
                //Go through and draw a triangle from the current point on the vector to the next and then the next and just have set z values for them as I am nto tracking the depth of the person.
                
                ofJson pt;
                pt["x"] = vec[r].x;
                pt["y"]=vec[r].y;
                pt["z"]=200;
                stroke.push_back(pt);
               
               
                
                //cout<<stroke.size()<<endl;
                
                ofDrawTriangle(vec[r].x,vec[r].y,200,vec[(r+1)].x,vec[(r+1)].y,300, vec[(r+2)].x,vec[(r+2)].y,100);
                
             
                
            }
        }
       // cout<<vecCopy.size()<<endl;
       
        ofTranslate(0,-200,0);
        if (vecCopy.size()>4) //if there is something in the vector.
        {

            for (int r=0; r<vecCopy.size()-4; r++)
            {
                if (ofGetElapsedTimef()>10.0)
                {

//                    ofScale(8, -8, -8);
                    ofNoFill();
                    int k=vecCopy.size()-1;
                    float value=255.0/max(vecCopy[k].x,vecCopy[k].y);
                     ofSetColor(ofRandom(0,255), uint8(value*vecCopy[k].y),uint8(value*vecCopy[k].x)); //Blue
                     ofDrawTriangle(vecCopy[r].x,vecCopy[r].y,200,vecCopy[(r+1)].x,vecCopy[(r+1)].y,300, vecCopy[(r+2)].x,vecCopy[(r+2)].y,100);
                }
            }
        }
    }
    
  


else//if not drawing anything then clear the vectors.
{
  
    vec.clear();
    
    vecCopy.clear();
}
}
        //Code to fill the vector based on the person.
        
        
        
        //Set size of vector array have a current index instead of vector overwriting existing value
        //need to iterate background until index -- go through full index
//        for(int i=0; i<contourFinder.size(); i++)
//        {
//            //Should only be ran one time right at the beginning
//            if (vec.empty())
//            {
//                vec.clear();
//                vec.push_back(ofVec3f(0,0,0));
//            }
//
//            for (int i=vec.size(); i>vec.size(); i--)
//            {
//                if (vec.size()-1>=numb_shapes)//I will erase from the beginning of the vector
//                {
//                    vec[i+1]=vec[0];
//                     cout<<vec[0]<<endl;
//                    //vec.erase(vec.begin());
//                }
//            }
//            //cout<<vec.size()<<endl;
//
//            ofPoint center = toOf(contourFinder.getCenter(i));
//
//            for(int j=vec.size()-1; j<numb_shapes; j++)
//            {
//                //go through and push back to the vector based on the center of the contour. This only works well really with one person in the frame, which I might need to go through and update later on.
//                ofVec2f temp_pos;
//                temp_pos.x=center.x;
//                temp_pos.y=center.y;
//                vec.push_back(temp_pos);
//
//                //cout<<vec.size()<<endl;
//
//                if (vec.size()>4) //if there is something in the vector.
//                {
//                    for (int r=0; r<vec.size()-4; r++)
//                    {
//                        //ofDrawCircle(vec[r].x-400,vec[r].y-400,10);
//
//                        ofNoFill();
//                        int k=vec.size()-1;
//                        float value=255.0/max(vec[k].x,vec[k].y); // Code to find the max of x and y points and map it to 0-255 for some color.
//                        //ofSetColor(uint8(value*vec[k].x), uint8(value*vec[k].y),uint8(value*vec[k].z)); //Rainbow
//                        //ofSetColor(uint8(value*vec[k].x), ofRandom(0,100),0); //RED
//
//                        //randomize the first value and then use the other two values to set the color.
//                        //Am not doing it with z value because I am having a hard time figuring out the z value as in tracking the depth fo the person and using that for the current z value. This is an interesting thing that I want to figure out.
//                        ofSetColor(ofRandom(0,255), uint8(value*vec[k].y),uint8(value*vec[k].x)); //Blue
//
//                        //ofTranslate(-100, -100);
//                        //ofDrawSphere(vec[r].x-500,vec[r].y-500,ofRandom(1000),10);
//                        //ofDrawLine(vec[r].x,vec[r].y,vec[(r+1)].x,vec[(r+1)].y);
//                        // ofDrawTriangle(vec[r].x,vec[r].y,ofRandom(200),vec[(r+1)].x,vec[(r+1)].y,ofRandom(100), vec[(r+2)].x,vec[(r+2)].y,ofRandom(-400));
//
//                        //Go through and draw a triangle from the current point on the vector to the next and then the next and just have set z values for them as I am nto tracking the depth of the person.
//
//                        ofDrawTriangle(vec[r].x,vec[r].y,200,vec[(r+1)].x,vec[(r+1)].y,300, vec[(r+2)].x,vec[(r+2)].y,100);
//
//                    }
//                }
//
//            }
//
//        }
//    }
//
//    else//if not drawing anything then clear the vectors.
//        {
//            vec.clear();
//            vecCopy.clear();
//        }



void ofApp::drawPointCloud()//all the code for the body is currently a mesh of triangles, I basically adapted the mesh example in the open frameworks book to work in live time with a person.
{
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.enableColors();
    
    ofBackground(0);
    float connectionDistance = 20; //connectionDistance = 45 //Triangles:25
    float farDistance=500;
    mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    
    int step = 7; //Lines=7
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {//Adding vertexes as well as setting colors from the update code based on the depth values of the person.
            if(kinect.getDistanceAt(x, y) > 800 && kinect.getDistanceAt(x, y)< 1400) {
                
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
                
                if(kinect.getDistanceAt(x, y) > 800 && kinect.getDistanceAt(x, y)< 1000)
                {
                    //ofSetColor(0,255,0);
                    mesh.addColor(col);
                    
                }
                else if(kinect.getDistanceAt(x, y) > 1000 && kinect.getDistanceAt(x, y)< 1200)
                {
                    mesh.addColor(col2);
                }
                else if(kinect.getDistanceAt(x, y) > 1200 && kinect.getDistanceAt(x, y)< 1400)
                {
                    mesh.addColor(col3);
                }
                
                else if(kinect.getDistanceAt(x, y) > 1100 && kinect.getDistanceAt(x, y)< 1200)
                {
                    mesh.addColor(col4);
                }
                else if(kinect.getDistanceAt(x, y) > 1200 && kinect.getDistanceAt(x, y)< 1400)
                {
                    mesh.addColor(col5);
                
                }
            }
        }
    }
    
    glPointSize(6);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, 0); // center the points a bit
    ofEnableDepthTest();
    
    int numVerts = mesh.getNumVertices(); //I am still using the code for connecting lines, instead of the code for connecting triangles, which im not really sure how to do I tried adding a third distance but couldn't get it running.
    for (int a=0; a<numVerts; ++a) {
        ofVec3f verta = mesh.getVertex(a);
        for (int b=a+1; b<numVerts; ++b) {
            ofVec3f vertb = mesh.getVertex(b);
            
            float distance = verta.distance(vertb);
            
            // if (distance <= connectionDistance || farDistance>=connectionDistance)
            if (distance <= connectionDistance)
            {
                // In OF_PRIMITIVE_LINES, every pair of vertices or indices will be
                // connected to form a line
                mesh.addIndex(a);
                mesh.addIndex(b);
                // mesh.addIndex(c);
            }
            // }
            
        }
    }
    //Code to see if I could get the person to rotate but is to no avail yet.
//    meshCentroid = mesh.getCentroid();
//    for (int i=0; i<numVerts; ++i) {
//        ofVec3f vert = mesh.getVertex(i);
//        float distance = vert.distance(meshCentroid);
//        float angle = atan2(vert.y-meshCentroid.y, vert.x-meshCentroid.x);
//        distances.push_back(distance);
//        angles.push_back(angle);
    //}
    
    // mesh.drawVertices();
    mesh.draw();
    ofDisableDepthTest();
    ofPopMatrix();
}

void ofApp::exit() {
    //kinect.setCameraTiltAngle(0); // zero the tilt on exit
    //kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){ //Kep pressed booleans and some code for thresholding kinect.
    
    switch (key) {
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case ' ':
            showLabels= !showLabels;
            
        case 'b':
            drawStuff=!drawStuff;
             startOrbitTime = ofGetElapsedTimef();
            
        case 'v':
            clearStuff=!clearStuff;
            
        case 'n':
            orbiting=!orbiting;
           
            mesh2 = meshCopy2; // This restores the mesh to its original values
            
        case 'x':
            drawBackground=!drawBackground;
       
        case's':
            ofSaveJson("drawing copy.json", js);
            js.push_back(stroke);
            stroke.clear();
            cout<<"saved"<<endl;
    }
    
}

//--------------------------------------------------------------
//void ofApp::keyReleased(int key){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mouseMoved(int x, int y ){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mouseDragged(int x, int y, int button){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mousePressed(int x, int y, int button){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mouseReleased(int x, int y, int button){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mouseEntered(int x, int y){
//
//}
//
////--------------------------------------------------------------
//void ofApp::mouseExited(int x, int y){
//
//}
//
////--------------------------------------------------------------
//void ofApp::windowResized(int w, int h){
//
//}
//
////--------------------------------------------------------------
//void ofApp::gotMessage(ofMessage msg){
//
//}
//
////--------------------------------------------------------------
//void ofApp::dragEvent(ofDragInfo dragInfo){
//
//}
