project(report)
cmake_minimum_required(VERSION 2.8)

# Set path for CMake
set(CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/CMakeLatex"
    ${CMAKE_MODULE_PATH}
)

include(UseLATEX)

file(GLOB_RECURSE input_files RELATIVE ${PROJECT_SOURCE_DIR} input/*.tex)

file(GLOB_RECURSE input_ijhr_files RELATIVE ${PROJECT_SOURCE_DIR} input-ijhr/*.tex)

ADD_LATEX_DOCUMENT(
    report.tex
    IMAGE_DIRS img
    INPUTS ${input_files}
    BIBFILES bib/report.bib bib/marco.bib
)

ADD_LATEX_DOCUMENT(
    ws-ijhr-tactile-exploration.tex
    IMAGE_DIRS img
    INPUTS ${input_ijhr_files}
    BIBFILES bib/report.bib bib/marco.bib
)
