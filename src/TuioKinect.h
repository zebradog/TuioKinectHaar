/*
 TuioKinect - A simple TUIO hand tracker for the Kinect 
 Copyright (c) 2010 Martin Kaltenbrunner <martin@tuio.org>
 
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

#ifndef _TUIO_KINECT
#define _TUIO_KINECT

// comment this line out if you don't wanna use the GUI
// you will need ofxSimpleGuiToo, ofxMSAInteractiveObject,1 & ofxXmlSettings
#define USE_GUI	

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCvKalman.h"
#include "ofxCvHaarFinder.h"
#include "TuioServer.h"

#ifdef USE_GUI 
#include "ofxSimpleGuiToo.h"
#endif

using namespace TUIO;

class TuioKinect : public ofBaseApp
{

	public:

		void setup();
		void update();
		void draw();
        void exit();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

		ofxKinect kinect;

		ofxCvGrayscaleImage	thresholdImage, prevThresholdImage;
		ofxCvGrayscaleImage	depthImage;
		ofxCvColorImage		colorImage;
		ofxCvGrayscaleImage	grayImage;
	    ofxCvGrayscaleImage	faceImage;
		
		ofxCvContourFinder 	contourFinder;
		TuioServer *tuioServer;

		int threshold;
		int thresholdOffset;
		float haarScale;
		int haarNeighbors;
		int haarMin;
	
		ofxCvHaarFinder haarFinder;
		ofRectangle face;
		bool foundFace;
	
		#ifdef USE_GUI 
			ofxSimpleGuiToo	gui;
		#endif

};

#endif
