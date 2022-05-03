/** @file Trajectory.cpp
 *  @brief Initializes the Trajectory.
 *
 *  @author Author
 */

#include <akrobat/Trajectory.h>

/** Overlaods Trajectory constructor.
*	@param tick present tick.
*	@param initAmpX initial amplitude for x (tripodAmpWidth/waveAmpWidth/rippleAmpWidth).
*	@param initAmpY initial amplitude for y (tripodAmpWidth/waveAmpWidth/rippleAmpWidth).
*	@param initAmpZ initial amplitude for z (tripodAmpWidth/waveAmpWidth/rippleAmpWidth).
*/
Trajectory::Trajectory() :
	tick(0),
	initAmpX(0.0f),
	initAmpY(0.0f),
	initAmpZ(0.0f)
{}