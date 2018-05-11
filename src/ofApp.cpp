#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

const float dyingTime = 1;

void Glow::setup(const cv::Rect& track) {
    color.setHsb(ofRandom(0, 255), 255, 255);
    cur = toOf(track).getCenter();
    smooth = cur;
}

void Glow::update(const cv::Rect& track) {
    cur = toOf(track).getCenter();
    smooth.interpolate(cur, .5);
    all.addVertex(smooth.x, smooth.y);
}

void Glow::kill() {
    float curTime = ofGetElapsedTimef();
    if(startedDying == 0) {
        startedDying = curTime;
    } else if(curTime - startedDying > dyingTime) {
        dead = true;
    }
}

void Glow::draw() {
    ofPushStyle();
    float size = 16;
    ofSetColor(255);
    if(startedDying) {
        ofSetColor(ofColor::red);
        size = ofMap(ofGetElapsedTimef() - startedDying, 0, dyingTime, size, 0, true);
    }
    ofNoFill();
    ofDrawCircle(cur, size);
    ofSetColor(color);
    all.draw();
    ofSetColor(255);
    ofDrawBitmapString(ofToString(label), cur);
    ofPopStyle();
}

void ofApp::setup() {
    int width = 640;
    int height = 480;
    cam.listDevices();
    cam.setDeviceID(0);
    cam.setup(width, height);
    flipped.allocate(width, height, OF_IMAGE_COLOR);
    masked.allocate(width, height);
    
    backgroundParameters.setName("background");
    backgroundParameters.add(resetBackground.set("reset", false));
    backgroundParameters.add(learningTime.set("learning time", 30, 0, 30));
    backgroundParameters.add(thresholdValue.set("threshold", 30, 0, 255));

    contourParameters.setName("contours");
    contourParameters.add(blobMinArea.set("min radius", max(width, height) / 50, 0, max(width, height) / 5));
    contourParameters.add(blobMaxArea.set("max radius", max(width, height) / 10, 0, max(width, height) / 5));
    contourParameters.add(contourThreshold.set("threshold", 25, 0, 255));
    
    trackerParameters.setName("tracker");
    trackerParameters.add(trackerPersistence.set("persistence", 15, 0, 600));
    trackerParameters.add(trackerMaximumDistance.set("maximum distance", 50, 0, width/4));

    mouseParameters.setName("mouse");
    colorParameters.setName("colors");
    for (int i = 0; i < 4; i++) {
        ofParameter<ofVec2f> mouse = ofVec2f(10 + 64 * i, height - 64);
        mouse.setName(ofToString(i));
        mouseParameters.add(mouse);
        
        ofParameter<ofColor> color = ofColor::fromHsb(32+(i*64), 255, 255);
        color.setName(ofToString(i));
        colorParameters.add(color);

        ofxCv::ContourFinder contourFinder = ofxCv::ContourFinder();
        contourFinders.push_back(contourFinder);
        ofxCv::RectTrackerFollower<Glow> tracker;
        trackers.push_back(tracker);
    }
    
    colorParameters.add(liveSampling.set("live sampling", false));
    lastIndex = 0;

    gui.setup();
    gui.add(backgroundParameters);
    gui.add(mouseParameters);
    gui.add(colorParameters);
    gui.add(contourParameters);
    gui.add(trackerParameters);
}

void ofApp::update() {
    cam.update();
    if (resetBackground) {
        background.reset();
        resetBackground = false;
    }
    if(cam.isFrameNew()) {
        background.setLearningTime(learningTime);
        background.setThresholdValue(thresholdValue);
        
        flipped.setFromPixels(cam.getPixels());
        flipped.mirror(false, true);
        //background.update(flipped, thresholded);
        //thresholded.update();

        // TODO: see if we can convolute to remove inner holes here?
	/*
        masked.setFromPixels(flipped.getPixels());
        const ofPixels thresholdedPixels = thresholded.getPixels();
	const int maskedSize = masked.getPixels().size();

	for (int i = 0; i < maskedSize; i++) {
	    masked.getPixels()[i] &= thresholdedPixels[i/3];
	}
	*/
        
        // TODO: checkout nAryMatIterator (p.83)
        for (int i = 0; i < trackers.size(); i++) {
            ofColor targetColor = colorParameters.getColor(ofToString(i));
            
            contourFinders[i].setTargetColor(targetColor, TRACK_COLOR_HS);
            contourFinders[i].setMinAreaRadius(blobMinArea);
            contourFinders[i].setMaxAreaRadius(blobMaxArea);
            contourFinders[i].setThreshold(contourThreshold);
            contourFinders[i].findContours(flipped);

            trackers[i].setPersistence(trackerPersistence);
            trackers[i].setMaximumDistance(trackerMaximumDistance);
            vector<cv::Rect> boundingRects = contourFinders[i].getBoundingRects();
            vector<cv::Rect> filteredBoundingRects;
            for(cv::Rect & boundingRect : boundingRects) {
                float ratio = (float) boundingRect.width / (float) boundingRect.height;
                if (ratio > 0.75 && ratio < 1.50) {
                    filteredBoundingRects.push_back(boundingRect);
                }
            }
            trackers[i].track(filteredBoundingRects);
        }
        if (liveSampling) {
            updateColors();
            ofParameterGroup & mouseParameters = ((ofParameterGroup &) gui.getGroup("mouse").getParameter());
            
            for (int i = 0; i < mouseParameters.size(); i++) {
                ofVec2f point = (ofVec2f) mouseParameters.getVec2f(ofToString(i));
                ofPushStyle();
                ofSetColor(255);
                ofDrawBitmapString("x", point.x, point.y);
                ofPopStyle();
            }
        }
    }
}

void ofApp::draw() {
    //thresholded.draw(640, 0);
    cam.draw(0, 0);
    flipped.draw(640, 0);
    
    int j = 0;
    for (ofxCv::RectTrackerFollower<Glow> & tracker : trackers) {
        //contourFinders[j].draw();
        vector<Glow>& followers = tracker.getFollowers();
        for(int i = 0; i < followers.size(); i++) {
            followers[i].draw();
        }
        j++;
    }

    for (int i = 0; i < mouseParameters.size(); i++) {
        ofVec2f point = (ofVec2f) mouseParameters.getVec2f(ofToString(i));
        ofDrawBitmapStringHighlight("x", point.x, point.y);
    }

    gui.draw();
}
void ofApp::updateColors() {
    ofParameterGroup & mouseParameters = ((ofParameterGroup &) gui.getGroup("mouse").getParameter());
    
    ofParameterGroup & colorParameters = ((ofParameterGroup &) gui.getGroup("colors").getParameter());
    
    for (int i = 0; i < mouseParameters.size(); i++) {
        ofVec2f point = (ofVec2f) mouseParameters.getVec2f(ofToString(i));
        ofColor color = cam.getPixels().getColor(point.x, point.y);
        colorParameters
            .getColor(ofToString(i))
            .set(color);
    }
}

void ofApp::mousePressed(int x, int y, int button) {
    if (x < 0 || x > 640 || y < 0 || y > 480) return;
    
    ofParameterGroup & mouseParameters = ((ofParameterGroup &) gui.getGroup("mouse").getParameter());
    
    mouseParameters.getVec2f(ofToString(lastIndex)) = ofVec2f(x, y);

    lastIndex = (lastIndex + 1) % mouseParameters.size();
    updateColors();
}
