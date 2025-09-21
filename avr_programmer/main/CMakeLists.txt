idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES hal protocols programmer utils ringbuffer
)
