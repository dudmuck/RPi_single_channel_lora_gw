%module scgw

%inline %{
    typedef unsigned char uint8_t;
%}

%{
/* Includes the header in the wrapper code */
#include "scgw.h"
%}

/*
%pythoncallback;
double f(double);
%nopythoncallback;

%ignore f;
*/
/* Parse the header file to generate wrappers */
%include "scgw.h"

