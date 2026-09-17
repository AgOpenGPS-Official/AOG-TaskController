add_executable(section_feedback_test section_feedback_test.cpp)
target_compile_features(section_feedback_test PRIVATE cxx_std_20)
target_include_directories(section_feedback_test PRIVATE ../include)
target_link_libraries(section_feedback_test PRIVATE isobus::Isobus
                                                    isobus::Utility)
add_test(NAME section_feedback COMMAND section_feedback_test)
