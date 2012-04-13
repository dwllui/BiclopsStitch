/*
 * timer.cpp
 *
 *  Created on: 20/07/2011
 *      Author: dennislui
 */

#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

timer::timer(){
}

timer::~timer(){

}

void timer::start(){

	#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0
		clock_gettime(CLOCK_REALTIME, &ts1);
	#else
		gettimeofday(&tv1,NULL);
		ts1.tv_sec = tv1.tv_sec;
		ts1.tv_nsec = tv1.tv_usec * 1000;
		//cout << "tv1.tv_sec: " << tv1.tv_sec << endl;
	#endif
}

uint64_t timer::stop(){

	uint64_t timeElapsed = 0;

	#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0
		clock_gettime(CLOCK_REALTIME, &ts2);
		//cerr << "Using clock_gettime" << endl;
	#else
		gettimeofday(&tv2, NULL);
		ts2.tv_sec = tv2.tv_sec;
		ts2.tv_nsec = tv2.tv_usec * 1000;
		//cout << "tv2.tv_sec: " << tv2.tv_sec << endl;
	#endif

	timeElapsed = (ts2.tv_nsec + ts2.tv_sec*1000000000)-(ts1.tv_nsec + ts1.tv_sec*1000000000);
	return timeElapsed;
	//cout << timeElapsed/1000000000.00 << endl;
	//cout << "FPS: " << (double)frameCount/(timeElapsed/1000000000.00) << endl;
}
