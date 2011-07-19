/*
 TuioKinect - A simple TUIO hand tracker for the Kinect 
 Copyright (c) 2010 Martin Kaltenbrunner <martin@tuio.org>
 Modified by Matt Cook <matt@lookitscook.com>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include "TuioKinect.h"

ofxCvKalman *tuioPointSmoothed[32];

TuioPoint updateKalman(int id, TuioPoint tp) {
	if (id>=16) return NULL;
	if(tuioPointSmoothed[id*2] == NULL) {
		tuioPointSmoothed[id*2] = new ofxCvKalman(tp.getX());
		tuioPointSmoothed[id*2+1] = new ofxCvKalman(tp.getY());
	} else {
		tp.update(tuioPointSmoothed[id*2]->correct(tp.getX()),tuioPointSmoothed[id*2+1]->correct(tp.getY()));
	}
	
	return tp;
}

void clearKalman(int id) {
	if (id>=16) return;
	if(tuioPointSmoothed[id*2]) {
		delete tuioPointSmoothed[id*2];
		tuioPointSmoothed[id*2] = NULL;
		delete tuioPointSmoothed[id*2+1];
		tuioPointSmoothed[id*2+1] = NULL;
	}
}


//--------------------------------------------------------------
void TuioKinect::setup()
{
	ofSetWindowTitle("TuioKinect");
	
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	
	depthImage.allocate(kinect.width, kinect.height);
	thresholdImage.allocate(kinect.width, kinect.height);
	prevThresholdImage.allocate(kinect.width, kinect.height);
	colorImage.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);

	threshold = 255;
	thresholdOffset = 8;

	haarScale = 1.38;
	haarNeighbors = 5;
	haarMin = 25;
	haarFinder.setup("HS.xml");
	
	foundFace = false;
	
	TuioTime::initSession();	
	tuioServer = new TuioServer("192.168.0.108",3333); //external client
	//tuioServer = new TuioServer(); //local client, localhost

	tuioServer->setSourceName("TuioKinect");
	tuioServer->enableObjectProfile(false);
	tuioServer->enableBlobProfile(false);
	
	for (int i=0;i<32;i++)
		tuioPointSmoothed[i] = NULL;
	
	#ifdef USE_GUI 
		gui.addSlider("threshold", threshold, 0, 255);
		gui.addSlider("offset (% threshold)", thresholdOffset, -25, 25);
		gui.addSlider("haar scale", haarScale, 1.01, 2.0);
		gui.addSlider("haar neighbors", haarNeighbors, 0, 20);
		gui.addSlider("haar min (% total)", haarMin, 1, 100);
		gui.setDefaultKeys(true);
		//gui.currentPage().setXMLName("config.xml");
		//gui.setAutoSave(false);
	#endif
	
		ofSetFrameRate(30);
	
}

//--------------------------------------------------------------
void TuioKinect::update()
{
	ofBackground(100, 100, 100);
	kinect.update();

	thresholdImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	thresholdImage.mirror(false, true);
	
	unsigned char * pix = thresholdImage.getPixels();
	int numPixels = thresholdImage.getWidth() * thresholdImage.getHeight()-1;
	
	depthImage.setFromPixels(pix, kinect.width, kinect.height);
	depthImage.flagImageChanged();
	
	colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
	colorImage.mirror(false, true);
	colorImage.convertToGrayscalePlanarImage(grayImage, 0);
	
	haarFinder.setScaleHaar(haarScale);
	haarFinder.setNeighbors(haarNeighbors);
	haarFinder.findHaarObjects(grayImage,kinect.width*haarMin*.01,kinect.height*haarMin*.01);
	int newThreshold = 0;
	for(int i = 0; i < haarFinder.blobs.size(); i++) {
		
		ofRectangle cur = haarFinder.blobs[i].boundingRect;
		depthImage.setROI(cur);
		faceImage.clear();
		faceImage.allocate(cur.width, cur.height);
		faceImage.setFromPixels(depthImage.getRoiPixels(),cur.width,cur.height);
		depthImage.resetROI();
		
		int numBins = 256;
		float range[] = {0, 255};
		float *ranges[] = { range };
		float max_value = 0, min_value = 0;
		int max_idx = 0, min_idx = 0;
		
		CvHistogram *hist = cvCreateHist(1, &numBins, CV_HIST_ARRAY,ranges,1);
		cvClearHist(hist);
		IplImage* faceCvImage = faceImage.getCvImage();
		cvCalcHist(&faceCvImage, hist, 0, 0);
		*(cvGetHistValue_1D(hist,0)) = 0; //discard the holes
		for(int i = threshold * (1 + .01 * thresholdOffset); i < 256; i++){
			*(cvGetHistValue_1D(hist,i)) = 0; //discard area in front threshold
		}
		
		cvGetMinMaxHistValue( hist, &min_value, &max_value, &min_idx, &max_idx);
		
		if(max_idx > newThreshold){ //only take closest face
			newThreshold = max_idx; 
			face.x = cur.x;
			face.y = cur.y;
			face.width = cur.width;
			face.height = cur.height;
			foundFace = true;
		}

	}
	if(newThreshold > 0) threshold = newThreshold;
	
	for(int i = numPixels; i > 0 ; i--){ 
		pix[i] = pix[i] > threshold * (1 + .01 * thresholdOffset) ? 255 : 0;
	}

	//update the cv image
	thresholdImage.flagImageChanged();
	
	contourFinder.findContours(thresholdImage, 900, (kinect.width*kinect.height)/8, 2, false);
	
	TuioTime frameTime = TuioTime::getSessionTime();
	tuioServer->initFrame(frameTime);
	
	std::vector<ofxCvBlob>::iterator blob;
	for (blob=contourFinder.blobs.begin(); blob!= contourFinder.blobs.end(); blob++) {
		float xpos = (*blob).centroid.x;
		float ypos = (*blob).centroid.y;
				
		TuioPoint tp(xpos/kinect.width,ypos/kinect.height);
		
		//if ((tp.getY() > 0.8) && (tp.getX()>0.25) && (tp.getX()<0.75)) continue;
		
		TuioCursor *tcur = tuioServer->getClosestTuioCursor(tp.getX(),tp.getY());
		if ((tcur==NULL) || (tcur->getDistance(&tp)>0.2)) { 
			tcur = tuioServer->addTuioCursor(tp.getX(), tp.getY());
			updateKalman(tcur->getCursorID(),tcur);
		} else {
			TuioPoint kp = updateKalman(tcur->getCursorID(),tp);
			tuioServer->updateTuioCursor(tcur, kp.getX(), kp.getY());
		}
	}

	tuioServer->stopUntouchedMovingCursors();
	
	std::list<TuioCursor*> dead_cursor_list = tuioServer->getUntouchedCursors();
	std::list<TuioCursor*>::iterator dead_cursor;
	for (dead_cursor=dead_cursor_list.begin(); dead_cursor!= dead_cursor_list.end(); dead_cursor++) {
		clearKalman((*dead_cursor)->getCursorID());
	}
	
	tuioServer->removeUntouchedStoppedCursors();
	tuioServer->commitFrame();
}

//--------------------------------------------------------------
void TuioKinect::draw()
{
	ofSetColor(255, 255, 255);
	ofFill();
	
	depthImage.draw(15, 15, 400, 300);
	grayImage.draw(425, 15, 400, 300);
	thresholdImage.draw(15, 325, 400, 300);
	contourFinder.draw(15, 325, 400, 300);

	//draw hand circles
	ofSetColor(255,255, 255);
	ofRect(425, 325, 400, 300);
	std::list<TuioCursor*> alive_cursor_list = tuioServer->getTuioCursors();
	std::list<TuioCursor*>::iterator alive_cursor;
	for (alive_cursor=alive_cursor_list.begin(); alive_cursor!= alive_cursor_list.end(); alive_cursor++) {
		TuioCursor *ac = (*alive_cursor);
		
		
		int xpos = 425+ac->getX()*400;
		int ypos = 325+ac->getY()*300;
		ofSetColor(0, 0, 0);
		ofCircle(xpos,ypos ,15);
		char idStr[32];
		sprintf(idStr,"%d",ac->getCursorID());
		ofDrawBitmapString(idStr, xpos+15, ypos+20);
		
	}
	
	//draw face boxes
	ofSetColor(200,255,0);
	ofNoFill();
	for(int i = 0; i < haarFinder.blobs.size(); i++) {
		ofRectangle cur = haarFinder.blobs[i].boundingRect;
		ofRect(425+(cur.x/kinect.width)*400,15+(cur.y/kinect.height)*300,(cur.width/kinect.width)*400,(cur.height/kinect.height)*300);
	}
	
	ofSetColor(255,100,0);
	ofRect(15+(face.x/kinect.width)*400,15+(face.y/kinect.height)*300,(face.width/kinect.width)*400,(face.height/kinect.height)*300);
	
	#ifdef USE_GUI 
		gui.draw();
	#endif
	
}

//--------------------------------------------------------------
void TuioKinect::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void TuioKinect::keyPressed (int key)
{}

//--------------------------------------------------------------
void TuioKinect::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void TuioKinect::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void TuioKinect::windowResized(int w, int h)
{}