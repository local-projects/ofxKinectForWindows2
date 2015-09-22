#pragma once

#include <string>
#include <Kinect.h>
#include <Kinect.Face.h>

namespace ofxKinectForWindows2 {
	using namespace std;
	namespace Source {
		class Base {
		public:
			virtual string getTypeName() const = 0;
			virtual void init(IKinectSensor *) = 0;
			virtual void update() = 0;
		};
	}
}