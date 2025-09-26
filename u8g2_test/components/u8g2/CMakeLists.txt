# components/u8g2/CMakeLists.txt

# Grab all .c files in this folder
file(GLOB U8G2_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

idf_component_register(
    SRCS ${U8G2_SOURCES}
    INCLUDE_DIRS "."
    REQUIRES driver
)
