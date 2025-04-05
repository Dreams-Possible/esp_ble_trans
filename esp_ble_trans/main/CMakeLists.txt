#all c file in this directory
file(GLOB SRCS "*.c")

#register component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)
