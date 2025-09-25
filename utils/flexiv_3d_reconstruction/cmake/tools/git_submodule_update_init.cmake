# This function runs "git submodule update --init", if needed.
# If the submodule has already been cloned, it will do nothing.
# This prevents local changes to the submodules from being lost.
#
# Example:
# 
#     git_submodule_update_init(cmake/tools)
#
function(git_submodule_update_init SUBMODULE_RELATIVE_PATH)

    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SUBMODULE_RELATIVE_PATH}/.git)
        execute_process(COMMAND git submodule update --init ${SUBMODULE_RELATIVE_PATH}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        )
    endif()

endfunction()

