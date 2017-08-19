/*
 * nis.h
 *
 *  Created on: Aug 15, 2017
 *      Author: magak
 */

#ifndef NIS_H_
#define NIS_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class nis {
private:

	/**
	* the threshold (X^2)0.50
	*/
	double threshold;

	/**
	* how many nis-es above the threshold
	*/
	int nisAboveThresholdCount;

	/**
	* how many nis-es
	*/
	int nisCount;

	int df_;

public:

	/**
	* Default constructor
	*/
	nis();

	/**
	* Constructor
	*/
	nis(int df);

	/**
	* Destructor
	*/
	virtual ~nis();

	/**
	* Calculates Normalized Innovations Squared
	* @param z_diff difference between predicted and measuered state vectors in measuruement space
	* @param S Covariance matrix
	*/
	double Calculate(const VectorXd& z_diff, const MatrixXd& S);


	/**
	* Calculates percentage of calculated nis, that exceeds the threshold (X^2)0.50
	*/
	double GetAboveThresholdPercentage();
};

#endif /* NIS_H_ */
