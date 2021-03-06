/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef APPS_ATLASROBOT_MYWINDOW_H_
#define APPS_ATLASROBOT_MYWINDOW_H_
#include <string>
#include "dart/gui/SimWindow.h"
#include "myUtils/SimFrame.h"
using namespace std;


namespace bioloidgp {
namespace robot {
class HumanoidController;
} // namespace robot
} // namespace bioloidgp

class MocapReader;
class CSerial;
class SupportInfo;
/// \brief class MyWindow
class MyWindow : public dart::gui::SimWindow
{
public:
    /// \brief Constructor
    explicit MyWindow(bioloidgp::robot::HumanoidController* _controller);

    /// \brief Destructor
    virtual ~MyWindow();

    // Documentation inherited
    virtual void timeStepping();
	virtual void draw();
    // Documentation inherited
    virtual void drawSkels();

    // Documentation inherited
    virtual void keyboard(unsigned char _key, int _x, int _y);
	virtual void displayTimer(int _val);
    void calculateInertia();
	void setController2(bioloidgp::robot::HumanoidController* _controller);
	Eigen::VectorXd samplePose(const vector<SimFrame>& frames, double time);
private:

	void readMovieFile(const string& fileName);
	void readMovieFile2(const string& fileName);
	Eigen::VectorXd align(const Eigen::VectorXd& pos, const vector<SimFrame>& movie);
	double mTime;   
	int mFrameCount;
	vector<SimFrame> mMovie;
	vector<SimFrame> mMovie2;
    bioloidgp::robot::HumanoidController* mController;	
	bioloidgp::robot::HumanoidController* mController2;
	Eigen::Isometry3d mOffsetTransform;
};

#endif  // APPS_ATLASROBOT_MYWINDOW_H_
