#include <stdio.h>
#include <math.h>
#include <stdlib.h>

int hue;
int saturation;
int brightness;
int rgb_colors[3];

void HSVtoRGB(int h_val, int s_val, int v_val)
{
    
}

int main(int argc, char *argv[])
{
    rgb_colors[0] = atoi(argv[0]);
    rgb_colors[1] = atoi(argv[1]);
    rgb_colors[2] = atoi(argv[2]);
    
    printf("R: %d G: %d B: %d\n", rgb_colors[0], rgb_colors[1], rgb_colors[2]);
    RGBtoHSV(rgb_colors[0], rgb_colors[1], rgb_colors[2]);
    printf("H: %d S: %d V: %d\n", hue, saturation, brightness);
    HSVtoRGB(hue, saturation, brightness);
    printf("R: %d G: %d B: %d\n", rgb_colors[0], rgb_colors[1], rgb_colors[2]);
    
    
}