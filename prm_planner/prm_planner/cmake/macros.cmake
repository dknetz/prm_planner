macro(add_module __LIB_NAME __LIB_SUB_DIR)
        set(LIB_NAME ${__LIB_NAME}_lib)
        #set(LIB_SUB_DIR ${__LIB_SUB_DIR} PARENT_SCOPE)
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/${__LIB_SUB_DIR})
endmacro()

