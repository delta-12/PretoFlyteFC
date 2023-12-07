set(sources "Sbus.c")
set(includes "include")

idf_component_register(
    SRCS ${sources}
    INCLUDE_DIRS ${includes}
    REQUIRES Sbus
    PRIV_REQUIRES driver
)