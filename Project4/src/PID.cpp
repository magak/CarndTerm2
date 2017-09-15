#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;

	reset();

	p_[0] = Kp;
	p_[1] = Kd;
	p_[2] = Ki;

	dp_[0] = p_[0]/4;
	dp_[1] = p_[1]/4;
	dp_[2] = p_[2]/4;

	bestError_ = 0;
	twiddleDone_ = false;
	twiddleIndex_ = 0;
	twiddleState_ = TwiddleState::NotInitialized;
}

void PID::reset(){
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	n_ = 0;
	totalCte_ = 0;
	firstCteReceived_ = false;
}

void PID::UpdateError(double cte) {
	n_ += 1;
	if(!firstCteReceived_)
		p_error = cte;

	firstCteReceived_ = true;

	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	if(n_ > 100){
		totalCte_ += cte*cte;
	}

	controlValue_ = -p_[0] * cte - p_[1] * d_error - p_[2] * i_error;
}

double PID::TotalError() {
	return totalCte_/n_;
}

double PID::GetControlValue(){
	return controlValue_;
}

double PID::getDpSum(){
	return dp_[0]+dp_[1]+dp_[2];
}

bool PID::Twiddle(double tol = 0.002){
	if(n_ < 2700){
		return false;
	}

	if(getDpSum() > tol){
		if(twiddleState_ == TwiddleState::NotInitialized){
			bestError_ = TotalError();
			twiddleState_ = TwiddleState::BeforeDpIncrease;
		}

		if(twiddleState_ == TwiddleState::BeforeDpIncrease){
			p_[twiddleIndex_] += dp_[twiddleIndex_];
			twiddleState_ = TwiddleState::AfterDpIncrease;
		}
		else if(twiddleState_ == TwiddleState::AfterDpIncrease){
			double error = TotalError();
			if(error < bestError_){
				bestError_ = error;
				dp_[twiddleIndex_] *= 1.1;
				twiddleState_ = TwiddleState::BeforeDpIncrease;
				twiddleIndex_ = (twiddleIndex_+1) % 3;
			}
			else {
				p_[twiddleIndex_] -= 2*dp_[twiddleIndex_];
				twiddleState_ = TwiddleState::AfterDpDecrease;
			}
		}
		else if(twiddleState_ == TwiddleState::AfterDpDecrease){
			double error = TotalError();
			if(error < bestError_){
				bestError_ = error;
				dp_[twiddleIndex_] *= 1.1;
			}
			else{
				p_[twiddleIndex_] += dp_[twiddleIndex_];
				dp_[twiddleIndex_] *= 0.9;
			}
			twiddleState_ = TwiddleState::BeforeDpIncrease;
			twiddleIndex_ = (twiddleIndex_+1) % 3;
		}

		reset();
	}
	else{
		twiddleDone_ = true;
	}

	std::cout << "dp1=" << dp_[0] << " dp2=" << dp_[1] << " dp3=" << dp_[2] << endl;
	std::cout << "p1=" << p_[0] << " p2=" << p_[1] << " p3=" << p_[2] << endl;
	std::cout << "error=" << TotalError() << endl;

	return true;
}
