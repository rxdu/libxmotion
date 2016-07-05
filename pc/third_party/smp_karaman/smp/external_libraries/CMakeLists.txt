# Add libraries
set(SMP_KARAMAN_DEP_LIB_SRC
    halton/halton.C
    inc_mu_mc/ms.cpp
    inc_mu_mc/pt.cpp
    kdtree/kdtree.c
)
add_library(smp_karaman_dep_lib STATIC ${SMP_KARAMAN_DEP_LIB_SRC})
