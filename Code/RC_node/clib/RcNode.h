#ifndef RC_NODE_H
#define RC_NODE_H

/// Store some 16-bit bitfields for tracking the system status and various erros.
/// These have been declared in Node.h
// status tracks various statuses of the node.
//  * bit 0: Manual control enabled.

// errors tracks the various flags that can put the node into a reset state.
//  * bit 0: eStop has been pushed.
//  * bit 1: rudder is calibrating
//  * bit 2: rudder is uncalibrated
//  * bit 3: the ECAN peripheral has reached an error state for transmission
//  * bit 4: the ECAN peripheral has reached an error state for reception

#endif // RC_NODE_H