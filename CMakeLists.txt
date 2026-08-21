idf_component_register(
    SRCS "src/core.c"
         "src/io.c"
         "src/health.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    PRIV_REQUIRES driver
)
