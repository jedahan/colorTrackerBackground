#pragma once

#include "ofMain.h"
#include "ofxCv.h"

class Glow : public ofxCv::RectFollower {
protected:
    ofColor color;
    float startedDying;
    ofPolyline all;
public:
    ofVec2f cur, smooth;
    Glow()
    :startedDying(0) {
    }
    void setup(const cv::Rect& track);
    void update(const cv::Rect& track);
    void kill();
    void draw();
};
