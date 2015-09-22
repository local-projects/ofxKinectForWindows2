#pragma once

#include "Joint.h"

#include <Kinect.h>
#include <Kinect.Face.h>

namespace ofxKinectForWindows2 {
	namespace Data {
		class Body {
		public:
			int bodyId;
			int trackingId;
			bool tracked;

			int faceId;
			int faceTrackingId;
			bool faceTracked;
			bool faceValid;

			float animationUnits[FaceShapeAnimations_Count];
			float deformationUnits[FaceShapeDeformations_Count];
			
			ofRectangle faceBoundingBox;
			ofQuaternion faceOrientation;

			HandState leftHandState;
			HandState rightHandState;
			std::map<JointType, Joint> joints;
			std::map<Activity, DetectionResult> activity;

			void drawWorld();
			void clear();

			static const std::vector<pair<JointType, JointType> > & getBonesAtlas();

		protected:
			static void initBonesAtlas();
			static vector<pair<JointType, JointType> > * bonesAtlas;

		};
	}
}