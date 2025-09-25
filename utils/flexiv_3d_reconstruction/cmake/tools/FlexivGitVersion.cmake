# This macro will execute a series of git commands to define the following variables
#    - GIT_BRANCH
#    - GIT_COMMIT_HASH

macro(FlexivGitVersion)

    # Get the current working branch
    execute_process(
      COMMAND git rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      OUTPUT_VARIABLE "${PROJECT_NAME}_GIT_BRANCH"
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Get the latest abbreviated commit hash of the working branch
    execute_process(
      COMMAND git rev-parse HEAD
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      OUTPUT_VARIABLE "${PROJECT_NAME}_GIT_COMMIT_HASH"
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )

endmacro()
