/*
 * cmotionHelper.h
 *
 *  Created on: 10.01.2015
 *      Author: cognition
 */


#ifndef HUMAN_COGNITION_NODE_RECEIVER_CMOTIONHELPER_H_
#define HUMAN_COGNITION_NODE_RECEIVER_CMOTIONHELPER_H_

#include <stdio.h>

class  cmotionHelper {
	public:

	static bool isContains(int arr[], int value, int length)
	{
		return (getIndex (arr, value, length ) > 0);
	}

	static int getIndex (int arr[], int value, int length)
	{

		for (int i = 0 ; i < length ; i++)
		{
			if(arr[i] == value)
				return i;
		}
		return -1;
	}
};

#endif /* HUMAN_COGNITION_NODE_RECEIVER_CMOTIONHELPER_H_ */
