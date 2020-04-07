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
    nearThreshold = 280; //230; //255
    farThreshold = 190; //70;
    
    // Zero the tilt on startup
    //angle = 0;
    // kinect.setCameraTiltAngle(angle);
    
    // Start from the front
    bDrawPointCloud = false;
    clearStuff=false;
    rotating=false;
    
//
//    col.r=0;
//    col.g=255;
//    col.b=0;
    
    //float accelh=1;
    
    // Allocate images
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshNear.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshFar.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayPreprocImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

    contourFinder.setFindHoles(false);
  
    //contourFinder.setSimplify(true);
    
    ofSetFrameRate(60);
    showLabels = true;
    
    contourFinder.setMinAreaRadius(60);
    contourFinder.setMaxAreaRadius(600);
    contourFinder.setThreshold(205);
    // wait for half a second before forgetting something
    contourFinder.getTracker().setPersistence(100);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    
    minDistance=10;
    
    thresh1=800;
    thresh2=900;
    thresh3=1000;
    thresh4=1200;
    thresh5=1400;
    
    colThresh1=400;
    colThresh2=1000;
    colThresh3=1200;
    colThresh4=1400;
    colThresh5=colThresh3-100;
    
    
     mainThresh1=400;
     mainThresh2=1400;
    
    
    ofSetVerticalSync(true);
    
    // open an outgoing connection to HOST:PORT
   
//    sender.setup("10.0.0.213", 8888); // This is an arbitrary port number.
//
//    receiver.setup(8888); // You can use the same port, as we just saw.
//   //receiver.setup(PORT);

    //const std::string IP_OF_A = "192.168.1.3";//NightHawk
    
    const std::string IP_OF_A = "10.0.0.213";
  
    const uint16_t RECEIVING_PORT_ON_A = 8888;
    const uint16_t RECEIVING_PORT_ON_B = 8888;
    
    // Uncomment for machine A
    //    sender.setup(IP_OF_B, RECEIVING_PORT_ON_B);
    //    receiver.setup(RECEIVING_PORT_ON_A);
    //
    //        // Uncomment for machine B
  
    sender.setup(IP_OF_A, RECEIVING_PORT_ON_A);
    receiver.setup(RECEIVING_PORT_ON_B);
   
}




//--------------------------------------------------------------
void ofApp::update()
{
    
 
    float time= ofGetElapsedTimef();
    
    for(int i=0; i<2000; i++)
    {
       meshMode=20+20*sin(i+time);
    }
    
    for(int i=0; i<2000; i++)
    {
        //ofSetColor(127+127*sin(i*.01+time),127+127*sin(i*.012+time),127+127*sin(i*.014+time));
        //pathFromContour.setFillColor(127+127*sin(i*.01+time),127+127*sin(i*.012+time),127+127*sin(i*.014+time));
        pointSize = 10+10*sin(i+time);
        scaleMesh=3+3*sin(i+time);
        sinConnectionDistance=20+20*sin(i+time);
  
        //cout<<pointSize<<endl;
        //Color for the person on offset sin curves.
                col.r = 127+127*2*sin(i*.01+time);
                col.g = 127+127*sin(i*.012+time);
                col.b = 127+127*sin(i*.014+time);
       
                colCos.r = 127+127*2*cos(i*.01+time);
                colCos.g = 127+127*cos(i*.012+time);
                colCos.b = 127+127*cos(i*.014+time);
        
        
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
        dilate(grayImage,5);
       // dilate(grayImage);
        
        erode(grayImage,5);
      //  erode(grayImage);
        
        // Mark image as changed
        grayImage.update();

        contourFinder.findContours(grayImage);
        
//        ofxOscMessage m;
//        m.setAddress("Here");
//        m.addStringArg("taint");
//        sender.sendMessage(m);
        
        
//        for (int i=0; i<sendVec.size(); i++)
//        {
//                ofxOscMessage m;
//                m.setAddress("Point");
//                m.addFloatArg(sendVec[i].x);
//                m.addFloatArg(sendVec[i].y);
//                m.addFloatArg(sendVec[i].z);
//                cout<<sendVec[i]<<endl;
//                sender.sendMessage(m, false); // why set to false?
//
//        }
        
        
        for(int i=0; i<contourFinder.size();i++)
        {
            float area= contourFinder.getContourArea(i);
            ofxOscMessage m;
            m.setAddress("Area,size");
            m.addFloatArg(area);
            m.addFloatArg(contourFinder.size());
            sender.sendMessage(m, false);
            
            //cout<<m<<endl;pb
            //cout<< contourFinder.getContourArea(i)<<endl;
            
            //ofPoint average=contourFinder.getAverage(i);
            // cout<<average<<endl;
        }
        
    }
    
    while(receiver.hasWaitingMessages())
    {
        //cout<<"here"<<endl; // you should see this anytime a message is received.
        
       // cout<<"Here"<<endl;
        
        // get the next message
        ofxOscMessage m;
        receiver.getNextMessage(m); // the change here was just to a more modern method signature.
        
        
//        if (m.getAddress() == "Here")
//        {
        
//            string here=m.getArgAsString(0);
        
        //cout << "Got message from " << m.getRemoteHost() << ":" << m.getRemotePort() << m.getArgAsString(0) << endl;
        
        //cout<<here<<endl;
        
     //   }
        
        //        {
//        //Drawing goodness
        if (m.getAddress() == "Point")
       {
           // cout<<m<<endl;
        //}
//
            tempReciever.x=m.getArgAsFloat(0);
            tempReciever.y=m.getArgAsFloat(1);
            tempReciever.z=m.getArgAsFloat(2);
        
           //  cout<<tempReciever.z<<endl;
//
//
           // cout<<tempReciever<<endl;
           const std::size_t MAX_BUFFER_SIZE3 = 100;
           
           // This section fills the buffer.
           if (storage.size() < MAX_BUFFER_SIZE3)
           {
               storage.push_back(tempReciever);
               
               nextIndexToWrite3 = storage.size();
           }
           
           else // This section reuses positions once it is full.
           {
               storage[nextIndexToWrite3] = tempReciever;
           }
           
           // This section figures out the next write index and
           nextIndexToWrite3 = (nextIndexToWrite3 + 1) % MAX_BUFFER_SIZE3;
      }

        if( m.getAddress()=="Area,size")
        {
            tempArea=m.getArgAsFloat(0);
            tempSize=m.getArgAsInt(1);
          
          //  tempArea=ofClamp(tempArea,2000,7000);
            tempArea=ofMap(tempArea,2000,100000,0,7);
            tempScale=ofMap(tempArea,0,7,.5,3);
              //  cout<<tempArea<<endl;
          
        }
        
    }
    
    

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255, 255, 255);
   
    RectTracker& tracker = contourFinder.getTracker();
    
    easyCam.begin();
    //camera.begin();
    
    //Drawing the person.
    ofScale(tempScale,tempScale,tempScale);
    drawPointCloud();
    
    //ofScale(1,-1,-1);
    //contourFinder.draw();
    
   // ofScale(3,3,3);
   
    ofTranslate(0,500,500);
  
    drawShapes();
    

     easyCam.end();
    
}


void ofApp::drawShapes()
{
     int counter=0;
    //boolean that is mapped to the b button to draw the triangles.
  if (drawStuff)
  {

            ofPoint center = toOf(contourFinder.getCenter(counter));
            //go through and push back to the vector based on the center of the contour. This only works well really with one person in the frame, which I might need to go through and update later on.
                ofVec3f temp_pos;
                temp_pos.x=center.x;
                temp_pos.y=center.y;
                temp_pos.z=kinect.getDistanceAt(center.x, center.y);
                temp_pos.z=ofClamp(temp_pos.z,800,1400);
                //cout<<"this comp"<<temp_pos.z<<endl;
      
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
       
      
  
      
    //  cout<<storage.size()<<endl;
      //storage.push_back(tempReciever);
      
      ofTranslate(-600, 1400, 0);
        ofScale(3, -6, -2);
        
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
               // ofDrawSphere(vec[r].x,vec[r].y,ofRandom(1000),10);
                //ofDrawLine(vec[r].x,vec[r].y,vec[(r+1)].x,vec[(r+1)].y);
                
                
                 //ofDrawTriangle(vec[r].x,vec[r].y,ofRandom(200),vec[(r+1)].x,vec[(r+1)].y,ofRandom(100), vec[(r+2)].x,vec[(r+2)].y,ofRandom(-400));
                
                
                /// If this is where you are sending you are sending A LOT of data
                ///
                ofxOscMessage m;
                m.setAddress("Point");
                m.addFloatArg(vec[r].x);
                m.addFloatArg(vec[r].y);
                m.addFloatArg(vec[r].z);
                sender.sendMessage(m, false); // why set to false?
                
                
               // cout<<m<<endl;
               // ofDrawTriangle(vec[r].x,vec[r].y,200,vec[(r+1)].x,vec[(r+1)].y,300, vec[(r+2)].x,vec[(r+2)].y,100);
                ofSetLineWidth(1);
                
         
                
            }
        }
      ofTranslate(100, 00);
      
      if (storage.size()>4) //if there is something in the vector.
      {
          for (int r=0; r<storage.size()-4; r++)
          {
              
              ofSetColor(colCos.r, colCos.b, colCos.g);
//              ofDrawSphere(storage[r].x, storage[r].y, storage[r].z,5);
//              ofDrawLine(storage[r].x, storage[r].y, storage[r].z, storage[r+1].x, storage[r+1].y, storage[r+1].z );
              
              //ofDrawSphere(storage[r].x, storage[r].y,700,5);
              
            
              // ofNoFill();
              //storage[r].z=ofMap(storage[r].z,0,1800,0,10);
             // cout<<storage[r].z<<endl;

              //ofDrawTriangle(storage[r].x, storage[r].y, storage[r].z, storage[r+1].x, storage[r+1].y, storage[r+1].z,storage[r+2].x, storage[r+2].y, storage[r+2].z);
              
             ofDrawTriangle(storage[r].x, storage[r].y, 800, storage[r+1].x, storage[r+1].y, 900,storage[r+2].x, storage[r+2].y, 850);
             
              
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
        
        
        




void ofApp::drawPointCloud()//all the code for the body is currently a mesh of triangles, I basically adapted the mesh example in the open frameworks book to work in live time with a person.
{
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.enableColors();
    
    ofBackground(0);
    
   // float connectionDistance = 20; //connectionDistance = 45 //Triangles:25
   float connectionDistance = 20;
   
    //float farDistance=500;
    
   // mesh.setMode(OF_PRIMITIVE_POINTS);
    
//    if(meshMode<7)
//    {
//         mesh.setMode(OF_PRIMITIVE_POINTS);
//    }
//
//    else if(meshMode>=7&&meshMode<=14)
//    {
//        mesh.setMode(OF_PRIMITIVE_LINES);
//    }
//    else if(meshMode>=14&&meshMode<=20)
//    {
//       mesh.setMode(OF_PRIMITIVE_TRIANGLES);
//    }
//    else if(meshMode>=15&&meshMode<=20)
//    {
//         mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
//    }
    

    
    int step = 7; //Lines=7
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {//Adding vertexes as well as setting colors from the update code based on the depth values of the person.
            if(kinect.getDistanceAt(x, y) > mainThresh1 && kinect.getDistanceAt(x, y)< mainThresh2) {
                
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
               // cout<<kinect.getDistanceAt(x, y)<<endl;
                
                if(kinect.getDistanceAt(x, y) > colThresh1 && kinect.getDistanceAt(x, y)< colThresh2)
                {
                    //ofSetColor(0,255,0);
                    mesh.addColor(col);
                   // mesh.setMode(OF_PRIMITIVE_POINTS);
                }
                
                else if(kinect.getDistanceAt(x, y) > colThresh2 && kinect.getDistanceAt(x, y)< colThresh3)
                {
                    mesh.addColor(col2);
                   // mesh.setMode(OF_PRIMITIVE_LINES);
                    
                }
                else if(kinect.getDistanceAt(x, y) > colThresh3 && kinect.getDistanceAt(x, y)< colThresh4)
                {
                    mesh.addColor(col3);
                      // mesh.setMode(OF_PRIMITIVE_TRIANGLES);
                }
                
                else if(kinect.getDistanceAt(x, y) > colThresh5 && kinect.getDistanceAt(x, y)< colThresh3)
                {
                    mesh.addColor(col4);
                     // mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
                }
                else if(kinect.getDistanceAt(x, y) > colThresh3 && kinect.getDistanceAt(x, y)< colThresh4)
                {
                    mesh.addColor(col5);
                    //mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
                
                }
            }
        }
    }
    //cout<<tempReciever.z<<endl;
    
    if (tempReciever.z<thresh1)
    {
        mesh.setMode(OF_PRIMITIVE_POINTS);
    }
    else  if (tempReciever.z>thresh1 && tempReciever.z<thresh2)
    {
        mesh.setMode(OF_PRIMITIVE_POINTS);
    }
    else  if (tempReciever.z>thresh2 && tempReciever.z<thresh3)
    {
        mesh.setMode(OF_PRIMITIVE_LINES);
    }
    else  if (tempReciever.z>thresh3 && tempReciever.z<thresh4)
    {
        mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    }
    else  if (tempReciever.z>thresh4 && tempReciever.z<thresh5)
    {
        mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
    }
    
    //glPointSize(5+pointSize);
    glPointSize(3+tempArea);
    
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
                
            }
            // }
            
        }
    }
    
    ofTranslate(-200,0,0);
    mesh.draw();
    ofDisableDepthTest();
    ofPopMatrix();
    
    if(tempSize>1)
    {
        
        ofScale(1,-1,-1);
        
        for(int i=0; i<tempSize;i++)
        {
            ofTranslate(600+i,0, 0);
            mesh.draw();
            ofDisableDepthTest();
            ofPopMatrix();
        }
    }
    
    

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
             //startOrbitTime = ofGetElapsedTimef();
            
        case 'v':
            clearStuff=!clearStuff;
            
        case 'n':
            orbiting=!orbiting;
           
            mesh2 = meshCopy2; // This restores the mesh to its original values
            
        case 'x':
            drawBackground=!drawBackground;
       
       // case's':
//            ofSaveJson("drawing copy.json", js);
//            js.push_back(stroke);
//            stroke.clear();
//            cout<<"saved"<<endl;
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
