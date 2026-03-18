# shared_types - Application-specific type definitions (events, states)
add_library(shared_types INTERFACE)
target_include_directories(shared_types INTERFACE include)
target_link_libraries(shared_types INTERFACE state_machine_framework_itf)
