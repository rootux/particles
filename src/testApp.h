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
	void testApp::updateParticleSystem(ofxParticleSystem* ps, ofVec2f gravitationCenter, ofVec2f leftHand, ofVec2f rightHand);

	ofxKFW2::Device		kinect;
	IBodyFrameReader*       m_pBodyFrameReader;
	ICoordinateMapper*      m_pCoordinateMapper;


	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	ofVec2f		lastHandPositionLeft[BODY_COUNT] = {};
	ofVec2f		lastHandPositionRight[BODY_COUNT] = {};
	ofVec2f		lastChestPositions[BODY_COUNT] = {};

	ofVec2f		lastKnownChestPosition[BODY_COUNT] = {};

	HandState   leftHandState;
	HandState   rightHandState;
	UINT64		lastBodyTrackingIds[BODY_COUNT] = { NULL,NULL,NULL,NULL,NULL,NULL };

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxParticleSystem particleSystems[BODY_COUNT] = {};

	int pmouseXLeft, pmouseYLeft;
	int pmouseXRight, pmouseYRight;
	ofVec2f pmouseVelLeft, pmouseVelRight;

	ofxParticleEmitter leftHandEmitter, rightHandEmitter, topEmitter, botEmitter, leftEmitter, rightEmitter;
	ofxParticleEmitter secondPersonEmitter;
	bool isLeftEmitterEnabled, isRightEmitterEnabled, isTopEmitterEnabled, isBottomEmitterEnabled;
	bool isTrackingSecondPerson;
	float rotAcc, gravAcc, fieldMult, drag;
	ofFloatPixels vectorField;

	ofTexture pTex, p1Tex, p2Tex;
	int displayMode;
};
