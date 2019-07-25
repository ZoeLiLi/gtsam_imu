#ifndef ALIGNMENT_H
#define ALIGNMENT_H
#include <iostream>
#include <string>
#include "constant.h"

namespace TADR
{
class SystemAlignment
{
public:
	SystemAlignment();
	virtual ~SystemAlignment();
	
public:
	bool Alignment(SensorData sensor_data, PositionInfo& position_info);

private:
	bool								align_from_gnss_;

};
}
#endif
