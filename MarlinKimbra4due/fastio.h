// **************************************************************************
//
// Description: Fast IO functions for Arduino Due
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef	_FASTIO_H
#define	_FASTIO_H

// --------------------------------------------------------------------------
// magic I/O routines
// --------------------------------------------------------------------------
#ifndef MASK
  #define MASK(PIN)  (1 << PIN)
#endif

/*
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define     READ(IO)        digitalRead(IO)
/// Write to a pin
#define     WRITE(IO, v)    digitalWrite(IO, v)
/// set pin as input
#define     SET_INPUT(IO)   pinMode (IO, INPUT)
/// set pin as input_pullup
#define     SET_PULLUP(IO)  pinMode (IO, INPUT_PULLUP)
/// set pin as output
#define     SET_OUTPUT(IO)  pinMode (IO, OUTPUT)
/// set pin as output


/// check if pin is an input
#define    GET_INPUT(IO)
/// check if pin is an output
#define    GET_OUTPUT(IO)

/// toggle a pin	
#define    TOGGLE(IO)      WRITE(IO, !READ(IO))

// Shorthand
#define OUT_WRITE(IO, v) { SET_OUTPUT(IO); WRITE(IO, v); }

#endif	//_FASTIO_H
