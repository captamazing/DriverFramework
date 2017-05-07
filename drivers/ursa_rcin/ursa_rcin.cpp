/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#include <string.h>
#include "DriverFramework.hpp"
#include "ursa_rcin.hpp"

using namespace DriverFramework;

#define POW2(_x) ((_x) * (_x))


int RC_IN::rc_in_init()
{

	return 0;
}

int RC_IN::start()
{
	return 0;
}

int RC_IN::stop()
{

	return 0;
}

void RC_IN::_measure()
{
	
}

int RC_IN::devRead(void *buf, size_t count)
{	
	return 0;
}