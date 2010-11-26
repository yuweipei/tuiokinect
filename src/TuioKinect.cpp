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
	grayImage.allocate(kinect.width, kinect.height);
	colorImage.allocate(kinect.width, kinect.height);
	redImage.allocate(kinect.width, kinect.height);

	nearThreshold = 50;
	farThreshold  = 200;
	//farThreshold  = 100;

	TuioTime::initSession();	
	tuioServer = new TuioServer();
	tuioServer->setSourceName("TuioKinect");
	tuioServer->enableObjectProfile(false);
	tuioServer->enableBlobProfile(false);
	
	for (int i=0;i<32;i++)
		tuioPointSmoothed[i] = NULL;

	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void TuioKinect::update()
{
	ofBackground(100, 100, 100);
	kinect.update();

	grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	grayImage.mirror(false, true);
	
	unsigned char * pix = grayImage.getPixels();
	int numPixels = grayImage.getWidth() * grayImage.getHeight()-1;
	
	depthImage.setFromPixels(pix, kinect.width, kinect.height);
	depthImage.flagImageChanged();
	
	colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
	colorImage.mirror(false, true);
	colorImage.convertToGrayscalePlanarImage(redImage, 0);


	
	for(int i = numPixels; i > 0 ; i--){
		if( pix[i] > nearThreshold && pix[i] < farThreshold ){
			pix[i] = 255;
		}else{
			pix[i] = 0;
		}
	}
	
	//update the cv image
	grayImage.flagImageChanged();
	
	unsigned char * red = redImage.getPixels();
	numPixels = redImage.getWidth() * redImage.getHeight();
	
	contourFinder.findContours(grayImage, 900, (kinect.width*kinect.height)/8, 20, false);

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

	depthImage.draw(15, 15, 400, 300);
	redImage.draw(425, 15, 400, 300);
	grayImage.draw(15, 325, 400, 300);
	contourFinder.draw(15, 325, 400, 300);

	ofSetColor(255, 255, 255);
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
	
	ofSetColor(255, 255, 255);
	char reportStr[1024];
	sprintf(reportStr, "set near threshold %i (press: + -)\nset far threshold %i (press: < >)\nnum blobs found %i, fps: %i", nearThreshold, farThreshold, (int)contourFinder.blobs.size(), (int)ofGetFrameRate());
	ofDrawBitmapString(reportStr, 20, 650);
}

//--------------------------------------------------------------
void TuioKinect::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void TuioKinect::keyPressed (int key)
{
	switch (key)
	{	
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
	}
}

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

