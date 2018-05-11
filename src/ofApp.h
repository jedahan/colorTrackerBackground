#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

class Glow : public ofxCv::RectFollower {
protected:
    ofColor color;
    ofVec2f cur, smooth;
    float startedDying;
    ofPolyline all;
public:
    Glow()
    :startedDying(0) {
    }
    void setup(const cv::Rect& track);
    void update(const cv::Rect& track);
    void kill();
    void draw();
};


class ofApp : public ofBaseApp {
public:
    void setup();
    void update();
    void draw();
    void mousePressed(int x, int y, int button);
    void updateColors();
	
    ofImage flipped;
    ofxCv::RunningBackground background;
    ofImage thresholded;
    ofxKinect kinect;

    int lastIndex = 0;

    ofxPanel gui;
    ofParameter<bool> liveSampling;
    ofParameter<bool> resetBackground;
    ofParameter<float> learningTime, thresholdValue, hueRange, contourThreshold, trackerPersistence, trackerMaximumDistance;
    ofParameter<float> blobMinArea, blobMaxArea;

    ofParameterGroup backgroundParameters;
    ofParameterGroup contourParameters;
    ofParameterGroup trackerParameters;
    ofParameterGroup mouseParameters;
    ofParameterGroup colorParameters;
    
    ofxCvColorImage masked;
    vector<ofxCv::ContourFinder> contourFinders;
    vector<ofxCv::RectTrackerFollower<Glow>> trackers;
};
