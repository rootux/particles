#pragma once

#include "ofMain.h"
#include "ofxParticles.h"
#include "ofxKinectForWindows2.h"

class testApp : public ofBaseApp {
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	void setup();
	void update();
	void draw();

	void testApp::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);
	ofVec2f testApp::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
	void testApp::DrawBody(const Joint* pJoints, const ofVec2f* pJointPoints);
	void testApp::DrawBone(const Joint* pJoints, const ofVec2f* pJointPoints, JointType joint0, JointType joint1);
	void testApp::DrawHand(HandState handState, const ofVec2f& handPosition);

	ofxKFW2::Device		kinect;
	IBodyFrameReader*       m_pBodyFrameReader;
	ICoordinateMapper*      m_pCoordinateMapper;


	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	ofVec2f		lastHandPositionLeft;
	ofVec2f		lastHandPositionRight;
	ofVec2f		lastChestPosition;

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxParticleSystem particleSystem;
	int pmouseX, pmouseY;
	ofVec2f pmouseVel;

	ofxParticleEmitter mouseEmitter, topEmitter, botEmitter, leftEmitter, rightEmitter;
	float rotAcc, gravAcc, fieldMult, drag;
	ofFloatPixels vectorField;

	ofTexture pTex, p1Tex, p2Tex;
	int displayMode;
};
