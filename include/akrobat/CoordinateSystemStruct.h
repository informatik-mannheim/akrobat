#ifndef COORDINATESYSTEMSTRUCT_H
#define COORDINATESYSTEMSTRUCT_H

#include <akrobat/akrobat_init.h>
#include <akrobat/LegStruct.h>

struct coordinateSystemStruct
{
	legStruct leg[numberOfLegs];	//array of legs
};

#endif // COORDINATESYSTEMSTRUCT_H