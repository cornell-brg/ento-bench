#include "pid.h"
#include <cstdio>

int main()
{
    PIDController<int> pid;

    pid.set_weights( 30, 3, 5 );
    pid.reset( 5000 );
    int pos = 1000;
    for ( int i = 0; i < 20; i++ ) {
        printf( "pos: %d\n", pos );
        int ctrl_sig = pid.update( pos );
        printf( "ctrl: %d\n", ctrl_sig );
        pos = pos + ( ctrl_sig / 20 );
        printf( "new pos: %d\n\n", pos );
    }


    PIDController<float> pidf;
    
    pidf.set_weights( 3.0, 0.3, 0.5 );
    pidf.reset( 500 );
    float fpos = 100;
    for ( int i = 0; i < 20; i++ ) {
        printf( "pos: %f\n", fpos );
        float ctrl_sig = pidf.update( fpos );
        printf( "ctrl: %f\n", ctrl_sig );
        fpos = fpos + ( ctrl_sig / 2.0 );
        printf( "new pos: %f\n\n", fpos );
    }
}
