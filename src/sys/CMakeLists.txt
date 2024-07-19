cmake_minimum_required(VERSION 3.16)

if(GEM5_ARMV7E-M_BUILD)
  set(SRC_FILES
    syscalls.c
  )

  add_library(sys STATIC
    ${SRC_FILES}
  )

  set_source_files_properties(syscalls.c
    PROPERTIES COMPILE_FLAGS "-fomit-frame-pointer")
  # Currently no libs to link for util_lib in gem5
else() # Native build

endif()


