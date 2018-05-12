#pragma once

#include "ofMain.h"
#include "ofxCv.h"

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
