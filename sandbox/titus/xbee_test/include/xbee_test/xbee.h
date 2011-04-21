/*
 * xbee.h
 *
 *  Created on: Jan 11, 2011
 *      Author: root
 */

#ifndef XBEE_H_
#define XBEE_H_

#include <stdio.h>
#include "xbee_test/serial.h"

class xbeePacket
{
public:
	struct msg_ATND {	int my;
						int sh;
						int sl;
						int ni;
						int pna;
						int dt;
						int stat;
						int prof_id;
						int manf_id;};

	static bool StartATMode( Serial::Serial * port );
	static int SendATND( Serial::Serial * port );
private:

};

#endif /* XBEE_H_ */
