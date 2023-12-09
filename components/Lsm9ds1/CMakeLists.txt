set(sources "Lsm9ds1.c")
set(includes "include")

idf_component_register(
    SRCS ${sources}
    INCLUDE_DIRS ${includes}
    REQUIRES Lsm9ds1
    PRIV_REQUIRES driver
)