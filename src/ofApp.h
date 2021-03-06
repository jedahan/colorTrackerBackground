#include "Glow.h"
#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxKinect.h"
#include "ofxLivingRoom.h"
#include "ofxOpenCv.h"

class ofApp : public ofBaseApp {
public:
  void setup();
  void update();
  void draw();
  void mousePressed(int x, int y, int button);
  void keyPressed(int key);
  void updateColors();
  void sendHues();

  ofxKinect kinect;

  int lastIndex = 0;

  ofxPanel gui;
  ofParameter<bool> liveSampling;
  ofParameter<bool> sending;
  ofParameter<float> depthMaximum;
  ofParameter<int> angle;
  ofParameter<float> hueRange;
  ofParameter<float> contourThreshold;
  ofParameter<float> trackerPersistence;
  ofParameter<float> trackerMaximumDistance;

  ofParameter<float> blobMinArea, blobMaxArea;

  ofParameterGroup depthParameters;
  ofParameterGroup contourParameters;
  ofParameterGroup trackerParameters;
  ofParameterGroup mouseParameters;
  ofParameterGroup colorParameters;

  ofxCvColorImage colorImage;
  ofxCvGrayscaleImage depthImage;
  ofxCvGrayscaleImage hue, sat, bri;
  ofxCvColorImage thresholdedColorImage;

  vector<ofxCv::ContourFinder> contourFinders;
  vector<ofxCv::RectTrackerFollower<Glow>> trackers;

  ofxLivingRoom room;
};
