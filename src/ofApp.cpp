#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
  room.setup();
  kinect.setRegistration(true);
  kinect.setDepthClipping(500, 1000); // 50cm to 2m range
  kinect.init();
  kinect.open();

  if (kinect.isConnected()) {
    ofLogNotice() << "sensor-emitter dist: "
                  << kinect.getSensorEmitterDistance() << "cm";
    ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance()
                  << "cm";
    ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize()
                  << "mm";
    ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance()
                  << "mm";
  }

  colorImage.allocate(kinect.width, kinect.height);
  depthImage.allocate(kinect.width, kinect.height);
  hue.allocate(kinect.width, kinect.height);
  sat.allocate(kinect.width, kinect.height);
  bri.allocate(kinect.width, kinect.height);
  thresholdedColorImage.allocate(kinect.width, kinect.height);

  depthParameters.setName("depth");
  depthParameters.add(depthMaximum.set("maximum", 160, 0, 255));
  depthParameters.add(angle.set("angle", 0, -30, 30));

  contourParameters.setName("contours");
  contourParameters.add(
      blobMinArea.set("min radius", max(kinect.width, kinect.height) / 50, 0,
                      max(kinect.width, kinect.height) / 5));
  contourParameters.add(
      blobMaxArea.set("max radius", max(kinect.width, kinect.height) / 10, 0,
                      max(kinect.width, kinect.height) / 5));
  contourParameters.add(contourThreshold.set("threshold", 25, 0, 255));

  trackerParameters.setName("tracker");
  trackerParameters.add(trackerPersistence.set("persistence", 15, 0, 600));
  trackerParameters.add(
      trackerMaximumDistance.set("maximum distance", 50, 0, kinect.width / 4));

  mouseParameters.setName("mouse");
  colorParameters.setName("colors");
  for (unsigned int i = 0; i < 4; i++) {
    ofParameter<ofDefaultVec2> mouse =
        ofDefaultVec2(10 + 64 * i, kinect.height - 64);
    mouse.setName(ofToString(i));
    mouseParameters.add(mouse);

    ofParameter<ofColor> color = ofColor::fromHsb(32 + (i * 64), 255, 255);
    color.setName(ofToString(i));
    colorParameters.add(color);

    ofxCv::ContourFinder contourFinder = ofxCv::ContourFinder();
    contourFinders.push_back(contourFinder);
    ofxCv::RectTrackerFollower<Glow> tracker;
    trackers.push_back(tracker);
  }

  colorParameters.add(liveSampling.set("live sampling", false));
  lastIndex = 0;

  gui.setup("settings", "settings.json", 640 + 64 + (4 * 3), 64 + (4 * 3));
  gui.add(depthParameters);
  gui.add(mouseParameters);
  gui.add(colorParameters);
  gui.add(contourParameters);
  gui.add(trackerParameters);
  gui.getGroup("mouse").minimize();
  gui.getGroup("colors").minimize();
  gui.loadFromFile("settings.json");
}

void ofApp::update() {
  kinect.update();
  if (kinect.isFrameNew()) {
    colorImage.setFromPixels(kinect.getPixels());

    depthImage.setFromPixels(kinect.getDepthPixels());
    depthImage.threshold(depthMaximum);

    // extract the hue
    colorImage.convertRgbToHsv();
    colorImage.convertToGrayscalePlanarImages(hue, sat, bri);
    colorImage.convertHsvToRgb();
    // mask by the masked mask
    hue &= depthImage;
    sat &= depthImage;
    bri &= depthImage;
    thresholdedColorImage.setFromGrayscalePlanarImages(hue, sat, bri);
    thresholdedColorImage.convertHsvToRgb();

    for (unsigned int i = 0; i < trackers.size(); i++) {
      ofColor targetColor = colorParameters.getColor(ofToString(i));

      contourFinders[i].setTargetColor(targetColor, TRACK_COLOR_HS);
      contourFinders[i].setMinAreaRadius(blobMinArea);
      contourFinders[i].setMaxAreaRadius(blobMaxArea);
      contourFinders[i].setThreshold(contourThreshold);
      contourFinders[i].findContours(thresholdedColorImage);

      trackers[i].setPersistence(trackerPersistence);
      trackers[i].setMaximumDistance(trackerMaximumDistance);
      vector<cv::Rect> boundingRects = contourFinders[i].getBoundingRects();
      vector<cv::Rect> filteredBoundingRects;
      for (cv::Rect &boundingRect : boundingRects) {
        float ratio = (float)boundingRect.width / (float)boundingRect.height;
        if (ratio > 0.75 && ratio < 1.50) {
          filteredBoundingRects.push_back(boundingRect);
        }
      }
      trackers[i].track(filteredBoundingRects);
    }

    if (liveSampling.get()) {
      updateColors();

      ofPushStyle();
      ofSetColor(255);

      ofParameterGroup &mouseParameters =
          ((ofParameterGroup &)gui.getGroup("mouse").getParameter());
      for (unsigned int i = 0; i < mouseParameters.size(); i++) {
        ofDefaultVec2 point =
            (ofDefaultVec2)mouseParameters.getVec2f(ofToString(i));
        ofDrawBitmapString("x", point.x, point.y);
      }

      ofPopStyle();
    }
  }
}

void ofApp::draw() {
  colorImage.draw(0, 0);
  thresholdedColorImage.draw(kinect.width, 0);

  int j = 0;
  for (ofxCv::RectTrackerFollower<Glow> &tracker : trackers) {
    // contourFinders[j].draw();
    vector<Glow> &followers = tracker.getFollowers();
    for (unsigned int i = 0; i < followers.size(); i++) {
      followers[i].draw();
    }
    j++;
  }

  for (unsigned int i = 0; i < mouseParameters.size(); i++) {
    ofDefaultVec2 point =
        (ofDefaultVec2)mouseParameters.getVec2f(ofToString(i));
    ofDrawBitmapStringHighlight("x", point.x, point.y);
  }

  ofPushMatrix();
  ofPushStyle();
  int pad = 3;
  int width = 64;

  ofTranslate(640, 2 * pad);

  for (unsigned int i = 0; i < colorParameters.size() - 1; i++) {
    ofColor color = (ofColor)colorParameters.getColor(ofToString(i));
    ofTranslate(width + (4 * pad), 0);
    ofFill();
    ofSetColor(0);
    ofDrawRectangle(-pad, -pad, width + (2 * pad), width + (2 * pad));
    ofSetColor(color);
    ofDrawRectangle(0, 0, width, width);
  }
  ofPopStyle();
  ofPopMatrix();

  gui.draw();
}

void ofApp::updateColors() {
  ofParameterGroup &mouseParameters =
      ((ofParameterGroup &)gui.getGroup("mouse").getParameter());

  ofParameterGroup &colorParameters =
      ((ofParameterGroup &)gui.getGroup("colors").getParameter());

  for (unsigned int i = 0; i < mouseParameters.size(); i++) {
    ofDefaultVec2 point =
        (ofDefaultVec2)mouseParameters.getVec2f(ofToString(i));
    ofColor color = colorImage.getPixels().getColor(point.x, point.y);
    colorParameters.getColor(ofToString(i)).set(color);
  }
}

void ofApp::keyPressed(int key) {
  switch (key) {
  case OF_KEY_UP:
    angle = min(angle + 1, 30);
    kinect.setCameraTiltAngle(angle);
    break;
  case OF_KEY_DOWN:
    angle = max(angle - 1, -30);
    kinect.setCameraTiltAngle(angle);
    break;
  }
}

void ofApp::mousePressed(int x, int y, int button) {
  if (x > 0 && x < 640 && y > 0 && y < 480) {
    ofParameterGroup &mouseParameters =
        ((ofParameterGroup &)gui.getGroup("mouse").getParameter());

    mouseParameters.getVec2f(ofToString(lastIndex)) = ofDefaultVec2(x, y);

    lastIndex = (lastIndex + 1) % mouseParameters.size();
    updateColors();
  }
}
