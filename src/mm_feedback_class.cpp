#include "mm_feedback_class.h"
using namespace TADR;

MMFeedbackPara::MMFeedbackPara(boost::shared_ptr<SensorFactors> sensor_factor)
: sensor_factors_(sensor_factor)
{
}

MMFeedbackPara::~MMFeedbackPara()
{
}
