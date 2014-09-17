/*
 * Authors:
 * 	Christian Benz,
 * 	Hendrik Pfeifer,
 * 	Heiko Reinemuth,
 * 	Christoph Döringer
 */

#ifndef TIME_H_
#define TIME_H_

#include <boost/date_time/posix_time/posix_time.hpp>

/**
 *
 *
 */
std::string timeStampToTimeString(const ros::Time& timeStamp) {
	return to_iso_string(timeStamp.toBoost());
}

/**
 *
 *
 */
ros::Time timeStringToTimeStamp(const std::string& timeString) {
	return ros::Time::fromBoost(boost::posix_time::from_iso_string(timeString));
}

#endif

