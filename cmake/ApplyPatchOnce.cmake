# Applies PATCH_FILE to the current directory with git, unless it is already
# applied. FetchContent can run the patch step again on a source tree it already
# patched, which a plain `git apply` would fail on.
#
# Usage: cmake -DGIT_EXECUTABLE=git -DPATCH_FILE=x.patch -P ApplyPatchOnce.cmake

execute_process(
  COMMAND "${GIT_EXECUTABLE}" apply --reverse --check --ignore-whitespace
          "${PATCH_FILE}"
  RESULT_VARIABLE alreadyApplied
  OUTPUT_QUIET ERROR_QUIET)
if(alreadyApplied EQUAL 0)
  message(STATUS "Patch already applied: ${PATCH_FILE}")
  return()
endif()

execute_process(COMMAND "${GIT_EXECUTABLE}" apply --ignore-whitespace
                        "${PATCH_FILE}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Failed to apply ${PATCH_FILE}")
endif()
message(STATUS "Applied patch: ${PATCH_FILE}")
