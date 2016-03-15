#include "testApp.h"


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

//--------------------------------------------------------------
void testApp::setup() {
	HRESULT hr;

	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	// Initialize the Kinect and get coordinate mapper and the body reader
	IBodyFrameSource* pBodyFrameSource = NULL;
	hr = kinect.getSensor()->get_CoordinateMapper(&m_pCoordinateMapper);
	ofLogNotice("Initialized Kinect");
	if (SUCCEEDED(hr))
	{
		hr = kinect.getSensor()->get_BodyFrameSource(&pBodyFrameSource);
	}

	if (SUCCEEDED(hr))
	{
		hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	}

	SafeRelease(pBodyFrameSource);





	ofSetFrameRate(60.0);
	ofBackground(0, 0, 0);
	leftHandEmitter.velSpread = ofVec3f(25.0, 25.0);
	leftHandEmitter.life = 4.0;
	leftHandEmitter.lifeSpread = 1;
	leftHandEmitter.numPars = 10;
	leftHandEmitter.color = ofColor(200, 200, 255);
	leftHandEmitter.colorSpread = ofColor(20, 20, 0);
	leftHandEmitter.size = 32;

	rightHandEmitter.velSpread = ofVec3f(25.0, 25.0);
	rightHandEmitter.life = 4.0;
	rightHandEmitter.lifeSpread = 1;
	rightHandEmitter.numPars = 10;
	rightHandEmitter.color = ofColor(200, 200, 255);
	rightHandEmitter.colorSpread = ofColor(20, 20, 0);
	rightHandEmitter.size = 32;

	leftEmitter.setPosition(ofVec3f(0, ofGetHeight() / 3));
	leftEmitter.setVelocity(ofVec3f(150.0, 0.0));
	leftEmitter.posSpread = ofVec3f(10, 10.0);
	leftEmitter.velSpread = ofVec3f(10.0, 10);
	leftEmitter.life = 20;
	leftEmitter.lifeSpread = 5.0;
	leftEmitter.numPars = 2;
	leftEmitter.color = ofColor(200, 100, 100);
	leftEmitter.colorSpread = ofColor(50, 50, 50);
	leftEmitter.size = 32;

	rightEmitter = leftEmitter;
	rightEmitter.setPosition(ofVec3f(ofGetWidth() - 1, ofGetHeight() * 2 / 3));
	rightEmitter.setVelocity(ofVec3f(-150.0, 0.0));
	rightEmitter.color = ofColor(100, 100, 200);
	rightEmitter.colorSpread = ofColor(50, 50, 50);

	topEmitter = leftEmitter;
	topEmitter.setPosition(ofVec3f(ofGetWidth() * 2 / 3, 0));
	topEmitter.setVelocity(ofVec3f(0.0, 150.0));
	topEmitter.color = ofColor(100, 200, 100);
	topEmitter.colorSpread = ofColor(50, 50, 50);

	botEmitter = leftEmitter;
	botEmitter.setPosition(ofVec3f(ofGetWidth() / 3, ofGetHeight() - 1));
	botEmitter.setVelocity(ofVec3f(0.0, -150.0));
	botEmitter.color = ofColor(200, 200, 0);
	botEmitter.colorSpread = ofColor(50, 50, 0);

	vectorField.allocate(128, 128, 3);

	ofLoadImage(pTex, "p.png");
	ofLoadImage(p1Tex, "p1.png");
	ofLoadImage(p2Tex, "p2.png");

	rotAcc = 4500;
	gravAcc = 13500;
	drag = 0.5;
	fieldMult = 40.0;
	displayMode = 0;

	ofEnableBlendMode(OF_BLENDMODE_ADD);
}

//--------------------------------------------------------------
void testApp::update() {

	kinect.update();



	if (!m_pBodyFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	
	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;

		hr = pBodyFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr))
		{
			ProcessBody(nTime, BODY_COUNT, ppBodies);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);


	for (int y = 0; y < vectorField.getHeight(); y++)
		for (int x = 0; x< vectorField.getWidth(); x++) {
			int index = vectorField.getPixelIndex(x, y);
			float angle = ofNoise(x / (float)vectorField.getWidth()*4.0, y / (float)vectorField.getHeight()*4.0, ofGetElapsedTimef()*0.05)*TWO_PI*2.0;
			ofVec2f dir(cos(angle), sin(angle));
			dir.normalize().scale(ofNoise(x / (float)vectorField.getWidth()*4.0, y / (float)vectorField.getHeight()*4.0, ofGetElapsedTimef()*0.05 + 10.0));
			vectorField.setColor(x, y, ofColor_<float>(dir.x, dir.y, 0));
		}

	float dt = min(ofGetLastFrameTime(), 1.0 / 10.0);
	particleSystem.gravitateTo(lastChestPosition, gravAcc, 1, 10.0, false);
	//particleSystem.gravitateTo(ofPoint(ofGetWidth() / 2, ofGetHeight() / 2), gravAcc, 1, 10.0, false);
	particleSystem.rotateAround(ofPoint(ofGetWidth() / 2, ofGetHeight() / 2), rotAcc, 10.0, false);
	particleSystem.applyVectorField(vectorField.getPixels(), vectorField.getWidth(), vectorField.getHeight(), vectorField.getNumChannels(), ofGetWindowRect(), fieldMult);
	if (leftHandState == HandState_Closed) {
		particleSystem.gravitateTo(ofPoint(lastHandPositionLeft.x, lastHandPositionLeft.y), gravAcc, 1, 10.0, false);
	}

	if (rightHandState == HandState_Closed) {
		particleSystem.gravitateTo(ofPoint(lastHandPositionRight.x, lastHandPositionRight.y), gravAcc, 1, 10.0, false);
	}

	particleSystem.update(dt, drag);

	particleSystem.addParticles(leftEmitter);
	particleSystem.addParticles(rightEmitter);
	particleSystem.addParticles(topEmitter);
	particleSystem.addParticles(botEmitter);


	ofVec2f mouseVelLeft(lastHandPositionLeft.x - pmouseXLeft, lastHandPositionLeft.y - pmouseYLeft);
	mouseVelLeft *= 20.0;
	if (leftHandState == HandState_Open) {
		leftHandEmitter.setPosition(ofVec3f(lastHandPositionLeft.x, lastHandPositionLeft.y), ofVec3f(lastHandPositionLeft.x, lastHandPositionLeft.y));
		leftHandEmitter.posSpread = ofVec3f(10.0, 10.0, 0.0);
		leftHandEmitter.setVelocity(pmouseVelLeft, mouseVelLeft);
		particleSystem.addParticles(leftHandEmitter);
	}
	pmouseXLeft = lastHandPositionLeft.x;
	pmouseYLeft = lastHandPositionLeft.y;
	pmouseVelLeft = mouseVelLeft;


	ofVec2f mouseVelRight(lastHandPositionRight.x - pmouseXRight, lastHandPositionRight.y - pmouseYRight);
	mouseVelRight *= 20.0;
	if (rightHandState == HandState_Open) {
		rightHandEmitter.setPosition(ofVec3f(lastHandPositionRight.x, lastHandPositionRight.y), ofVec3f(lastHandPositionRight.x, lastHandPositionRight.y));
		rightHandEmitter.posSpread = ofVec3f(10.0, 10.0, 0.0);
		rightHandEmitter.setVelocity(pmouseVelRight, mouseVelRight);
		particleSystem.addParticles(rightHandEmitter);
	}
	pmouseXRight = lastHandPositionRight.x;
	pmouseYRight = lastHandPositionRight.y;
	pmouseVelRight = mouseVelRight;
}


/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void testApp::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{

	HRESULT hr;
	if (m_pCoordinateMapper)
	{
		for (int i = 0; i < nBodyCount; ++i)
		{
			IBody* pBody = ppBodies[i];
			if (pBody)
			{
				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);

				if (SUCCEEDED(hr) && bTracked)
				{
					Joint joints[JointType_Count];
					ofVec2f jointPoints[JointType_Count];
					leftHandState = HandState_Unknown;
					rightHandState = HandState_Unknown;

					pBody->get_HandLeftState(&leftHandState);
					pBody->get_HandRightState(&rightHandState);

					hr = pBody->GetJoints(_countof(joints), joints);
					if (SUCCEEDED(hr))
					{
						for (int j = 0; j < _countof(joints); ++j)
						{
							jointPoints[j] = BodyToScreen(joints[j].Position, 1920, 1080);
						}

						lastChestPosition = jointPoints[JointType_Neck];
						
						//DrawBody(joints, jointPoints);

						lastHandPositionLeft = jointPoints[JointType_HandLeft];
						lastHandPositionRight = jointPoints[JointType_HandRight];

					}
				}
			}
		}

		//hr = m_pRenderTarget->EndDraw();

		// Device lost, need to recreate the render target
		// We'll dispose it now and retry drawing
		if (D2DERR_RECREATE_TARGET == hr)
		{
			hr = S_OK;
			//DiscardDirect2DResources();
		}
	}

	if (!m_nStartTime)
	{
		m_nStartTime = nTime;
	}

	double fps = 0.0;

	LARGE_INTEGER qpcNow = { 0 };
	if (m_fFreq)
	{
		if (QueryPerformanceCounter(&qpcNow))
		{
			if (m_nLastCounter)
			{
				m_nFramesSinceUpdate++;
				fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
			}
		}
	}


}



/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
ofVec2f testApp::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

	return ofVec2f(screenPointX, screenPointY);
}


/// <summary>
/// Draws a body 
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void testApp::DrawBody(const Joint* pJoints, const ofVec2f* pJointPoints)
{
	// Draw the bones
	ofLogWarning("Draw Body");
	// Torso
	DrawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck);
	DrawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid);
	DrawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight);
	DrawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft);

	// Right Arm    
	DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
	DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
	DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
	DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);

	// Left Arm
	DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
	DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
	DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
	DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);

	// Right Leg
	DrawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight);
	DrawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight);
	DrawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight);

	// Left Leg
	DrawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft);
	DrawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft);
	DrawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft);

	// Draw the joints
	for (int i = 0; i < JointType_Count; ++i)
	{

		//ofEllipse ellipse = ofEllipse(pJointPoints[i], c_JointThickness, c_JointThickness);

		if (pJoints[i].TrackingState == TrackingState_Inferred)
		{
			//m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
		}
		else if (pJoints[i].TrackingState == TrackingState_Tracked)
		{
			//m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
		}
	}
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void testApp::DrawBone(const Joint* pJoints, const ofVec2f* pJointPoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		//m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness);
	}
	else
	{
		//m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness);
	}
}



//--------------------------------------------------------------
void testApp::draw() {
	if (ofGetKeyPressed('v')) {
		ofSetLineWidth(1.0);
		ofSetColor(80, 80, 80);
		ofPushMatrix();
		ofScale(ofGetWidth() / (float)vectorField.getWidth(), ofGetHeight() / (float)vectorField.getHeight());
		for (int y = 0; y < vectorField.getHeight(); y++)
			for (int x = 0; x< vectorField.getWidth(); x++) {
				ofColor_<float> c = vectorField.getColor(x, y);
				ofVec2f dir(c.r, c.g);

				ofLine(x, y, x + dir.x, y + dir.y);
			}
		ofPopMatrix();
	}

	ofNoFill();
	ofSetCircleResolution(180);
	ofSetColor(255, 102, 159, 255);
	ofCircle(ofGetWidth() / 2, ofGetHeight() / 2, sqrt(gravAcc));
	ofSetColor(0, 0, 180, 255);

	ofCircle(ofGetWidth() / 2, ofGetHeight() / 2, sqrt(rotAcc));

	ofSetLineWidth(2.0);
	if (displayMode == 1) {
		particleSystem.draw(pTex);
	}
	else if (displayMode == 2) {
		particleSystem.draw(p1Tex, p2Tex);
	}
	else {
		particleSystem.draw();
	}
	
	ofSetLineWidth(8.0);
	ofSetColor(101, 66, 138, 255);
	if (lastChestPosition.x > 0 && lastChestPosition.y > 0) {
		ofCircle(lastChestPosition.x, lastChestPosition.y, 60);
	}

	//Update rotation accelerator by hand distance

	if (lastHandPositionRight.x - lastHandPositionLeft.x >= 800) {
		gravAcc *= 1.03;
	}
	else if ((lastHandPositionRight.x - lastHandPositionLeft.x <= 400) && 
		(lastHandPositionRight.x - lastHandPositionLeft.x >= 200)) {
		if (gravAcc > 1.1)
			gravAcc /= 1.03;
	}
	

	ofSetLineWidth(2.0);

	ofSetColor(255, 255, 255);
	ofDrawBitmapString(ofToString(particleSystem.getNumParticles()) + "\n" + ofToString(ofGetFrameRate()) +
		"\n(G/g) gravitation: " + ofToString(gravAcc) +
		"\n(R/r) rotational acceleration: " + ofToString(rotAcc) +
		"\n(F/f) vector field multiplier: " + ofToString(fieldMult) +
		"\n(D/d) drag constant: " + ofToString(drag) +
		"\n(v) show vector field" +
		"\n(1-3) particle display modes", 20, 20);





}

//--------------------------------------------------------------
void testApp::keyPressed(int key) {
	switch (key) {
	case 'r':
		if (rotAcc > 1.1)
			rotAcc /= 1.1;
		break;
	case 'R':
		rotAcc *= 1.1;
		break;

	case 'g':
		if (gravAcc > 1.1)
			gravAcc /= 1.1;
		break;
	case 'G':
		gravAcc *= 1.1;
		break;

	case 'd':
		if (drag > 0.01)
			drag /= 1.01;
		break;
	case 'D':
		drag *= 1.01;
		if (drag > 1.0) drag = 1.0;
		break;
	case 'f':
		if (fieldMult > 0.1)
			fieldMult /= 1.1;
		break;
	case 'F':
		fieldMult *= 1.1;
		break;
	case '1':
		displayMode = 0;
		break;
	case '2':
		displayMode = 1;
		break;
	case '3':
		displayMode = 2;
		break;
	default:
		break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h) {
	leftEmitter.setPosition(ofVec3f(0, h / 3));
	rightEmitter.setPosition(ofVec3f(w - 1, h * 2 / 3));
	topEmitter.setPosition(ofVec3f(w * 2 / 3, 0));
	botEmitter.setPosition(ofVec3f(w / 3, h - 1));
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo) {

}
