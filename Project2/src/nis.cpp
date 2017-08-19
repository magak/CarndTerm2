#include "ukf.h"
#include "Eigen/Dense"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * default constructor
 */
nis::nis() {
}

/**
 * Initializes nis calculator
 */
nis::nis(int df) {
	df_ = df;

	if(df == 1) {
		threshold = 3.841;
	}
	else if(df == 2) {
		threshold = 5.991;
	}
	else if(df == 3) {
		threshold = 7.815;
	}
	else if(df == 4) {
		threshold = 9.488;
	}
	else if(df == 5) {
		threshold = 11.070;
	}
	else
	{
		throw runtime_error("Unexpected degree of freedom");
	}


	// initialize counters
	nisAboveThresholdCount = 0;
	nisCount = 0;
}

nis::~nis() {}

/**
* Calculates Normalized Innovations Squared
* @param z_diff difference between predicted and measuered state vectors in measuruement space
* @param S Covariance matrix
*/
double nis::Calculate(const VectorXd& z_diff, const MatrixXd& S){
	auto nis = z_diff.transpose()*S.inverse()*z_diff;
	if(nis > threshold){
		nisAboveThresholdCount++;
	}

	nisCount++;
}

/**
* Calculates percentage of calculated nis, that exceeds the threshold (X^2)0.50
*/
double nis::GetAboveThresholdPercentage(){
	return (double)nisAboveThresholdCount/nisCount;
}
