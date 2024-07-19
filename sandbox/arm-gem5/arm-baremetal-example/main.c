#include <stdarg.h>
#include <unistd.h>

#include <math.h>
#ifdef NANO_SPECS
#include <stdio.h>
//#include "syscalls.c"
#else
#include "syscalls.c"
void mini_printf(const char *format, ...)
{
  va_list args;
  va_start(args, format);

  while (*format != '\0')
  {
      if (*format == '%')
      {
        format++;
        switch (*format)
        {
          case 's':
          {
            char *str = va_arg(args, char*);
            while (*str)
            {
              write(1, str, 1);
              str++;
            }
            break;
          }
          case 'd':
          {
            int i = va_arg(args, int);
            char buf[10];
            int pos = 9;
            buf[pos--] = '\0';  // Null-terminate the string
            if (i == 0)
            {
                buf[pos--] = '0';
            }
            else
            {
              int negative = i < 0;
              if (negative)
              {
                i = -i;
              }
              while (i > 0)
              {
                buf[pos--] = (i % 10) + '0';
                i /= 10;
              }
              if (negative)
              {
                buf[pos--] = '-';
              }
            }
            write(1, &buf[pos + 1], 9 - pos);
            break;
          }
        }
      }
      else
      {
          write(1, format, 1);
      }
      format++;
  }

  va_end(args);
}
#endif



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

#ifdef NANO_SPECS
  printf("Hello gem5!\n"
         "a: %i, b: %i, c:%i\n"
         "d: %f, e: %f, f: %f\n", 
          a, b, c, d, e, f);
  //printf("Hello gem5!\n");
#else
  e += f;
  //mini_printf("Hello gem5!\n");
#endif

	return 0;
}

