#include <stdio.h>

int main(void)
{
	int a = 1;
	int b = 2;
	int c = 0;
	c = a + b;
  a += c;

	float d = 3.15;
	float e = 1.04;
	float f = 0.0;
	f = d + e;
	e += f;

	//printf("Hello gem5! c: %i, a: %i, f: %f, e: %f", c, a, f, e);

	return 0;
}

