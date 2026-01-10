list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)

idf_component_register(SRCS "mcp7940.c"
                    INCLUDE_DIRS "include")

include(eil_ci)
