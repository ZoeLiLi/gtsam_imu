#ifndef MMFEEDBACK_CLASS_H
#define MMFEEDBACK_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"

namespace TADR
{
class MMFeedbackPara
{
public:
	MMFeedbackPara(boost::shared_ptr<SensorFactors> sensor_factor);
	virtual ~MMFeedbackPara();

public:
private:
	boost::shared_ptr<SensorFactors>			sensor_factors_;
};
}
#endif
