/*
 * timer.h
 *
 *  Created on: 20/07/2011
 *      Author: dennislui
 */

#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0
#include<time.h>
#else
#include<sys/time.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include<sys/types.h>

using namespace std;


#ifndef TIMER_H_
#define TIMER_H_

class timer{
	public:
		timer();
		~timer();
		//starts the timer
		void start();
		//stops the timer and returns the elapsed time in nanoseconds
		uint64_t stop();

	private:
		timespec ts1,ts2;
		timeval tv1, tv2;
};


#endif /* TIMER_H_ */
