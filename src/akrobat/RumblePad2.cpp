/** @file RumblePad2.cpp
 *  @brief Initializes RumblePad 2 Controller.
 *
 *  @author Author
 */

#include <akrobat/RumblePad2.h>

/** Overlaods RublePad2 constructor.
*	@param speed Forward, backward, sideward movement.
*	@param bdR body rotation.
*	@param bdT body translation.
*/
RumblePad2::RumblePad2(): 
speed(0.0, 0.0, 0.0), 
bdR(0.0, 0.0, 0.0), 
bdT(0.0, 0.0, 0.0) 
{}