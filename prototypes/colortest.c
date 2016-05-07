#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* Convert HSV values to 0-255 RGB values. H ranges between 0-360, others between
   0 and 255. */
void HSVtoRGB(int h_val, int s_val, int v_val, int* r, int* g, int* b)
{
    int i;
	float f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}
/* Convert 0-255 RGB values to HSV. H ranges between 0-360, others between
   0 and 255. */
void RGBtoHSV(int r_val, int g_val, int b_val, int* h, int* s, int* v)
{
    float min, max, delta;
	min = MIN( r, g, b );
	max = MAX( r, g, b );
	*v = max;				// v
	delta = max - min;
	if( max != 0 )
		*s = delta / max;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		*s = 0;
		*h = -1;
		return;
	}
	if( r == max )
		*h = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		*h = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		*h = 4 + ( r - g ) / delta;	// between magenta & cyan
	*h *= 60;				// degrees
	if( *h < 0 )
		*h += 360;
}

int main(int argc, char *argv[])
{
    
    int hue;
    int saturation;
    int brightness;
    int r;
    int g;
    int b;
    
    r = atoi(argv[0]);
    g = atoi(argv[1]);
    b = atoi(argv[2]);
    
    printf("R: %d G: %d B: %d\n", r, g, b);
    RGBtoHSV(r, g, b, &hue, &saturation, &brightness);
    printf("H: %d S: %d V: %d\n", hue, saturation, brightness);
    HSVtoRGB(hue, saturation, brightness, &r, &g, &b);
    printf("R: %d G: %d B: %d\n", r, g, b);
    
    
}