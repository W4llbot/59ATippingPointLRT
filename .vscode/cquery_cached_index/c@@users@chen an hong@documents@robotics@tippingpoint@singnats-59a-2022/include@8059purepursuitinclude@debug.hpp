#ifndef _8059_PP_DEBUG_HPP_
#define _8059_PP_DEBUG_HPP_
#include <vector>
/**
 * DEBUG_MODE can be used to debug & test functions and tasks via the terminal.
 * 0: None
 * 1: Odometry (print Coordinates position)
 * 2: Encoders (print errorEncdL & errorEncdR)
 * 3: Power (print powerL & powerR)
 * 4: Raw encoder values (print raw encdL & encdR)
 */
#define DEBUG_MODE -1
void printVector(std::vector<double> vd);
void posPrintTerminal();
void posPrintMaster();
void encdPrintTerminal();
void Debug(void * ignore);
#endif
