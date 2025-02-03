.global _Reset
_Reset:
  ldr r3, =stack_top 
  mov sp, r3
  //LDR sp, =stack_top  This won't compile for CM4. Need to use a lo register.
  MOV  r8, #0xfef0
  MOVT r8, #0xbeff
	LDR r0, [r8]
  LDR r1, =0xbefffef4
  LDR r5, =main
  #orr r1, r1, #1	   // This won't compile for CM4.
	#add r1, r1, #1
  blx r5  					 // Ensure we go into thumb mode. 
  b exit_program

.global exit_program
exit_program:
  mov r0, #0
	ldr r7, =1
	svc #0
