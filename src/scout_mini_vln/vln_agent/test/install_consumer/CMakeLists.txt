cmake_minimum_required(VERSION 3.16)
project(vln_agent_install_consumer LANGUAGES CXX)

find_package(vln_agent REQUIRED)
add_executable(install_consumer main.cpp)
target_link_libraries(install_consumer vln_agent::vln_agent_tools)


add_executable(session_consumer session_main.cpp)
target_link_libraries(session_consumer vln_agent::vln_agent_session)
