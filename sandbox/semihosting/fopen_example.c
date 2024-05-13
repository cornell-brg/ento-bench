#include <stdio.h>

#include <stm32g4xx.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_rcc.h>
#include <systick_config.h>
#include <led_util.h>

extern void initialise_monitor_handles(void);

int main()
{
  initialise_monitor_handles();

  int status = 0;
  char buffer[100];

  printf("Opening a file from the host system.\n");
  
  // Hard-coded path with symlink works
  FILE *file = fopen("/Users/derinozturk/workspace/entomoton-bench/datasets/unittest/hello-world.txt", "r");

  // Tilde does not work. Cannot find file.
  //FILE *file = fopen("~/workspace/entomoton-bench/datasets/unittest/hello-world.txt", "r");
  if (file == NULL) {
      printf("Failed to open file.\n");
      status = 0;
  }
  else
  {
    if (fgets(buffer, sizeof(buffer), file) != NULL) {
        printf("File content: %s", buffer);
        status = 1;
    } else {
        printf("Failed to read from file.\n");
        status = 0;
    }
  }
  fclose(file);
  UserLED_Init();
  SystemCoreClockUpdate();
  if (status) SysTick_Setup(UserLED_Toggle, 2000); // Slow blink for success
  else SysTick_Setup(UserLED_Toggle, 100); // Fast blink for failure
  SysTick_Config(SystemCoreClock / 1000); // Enable SysTick_Interrupt

  while(1);
}

