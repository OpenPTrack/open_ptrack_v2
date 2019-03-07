#ifndef AR_DEVICE_REGISTRATION_ESIMATOR_HPP
#define AR_DEVICE_REGISTRATION_ESIMATOR_HPP


class ARDeviceRegistrationEstimator
{
private:
	std::vector<tf::Pose> arcoreWorldHistory;
	tf::Pose lastEstimate;	
};



#endif

