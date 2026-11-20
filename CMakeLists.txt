# SPDX-License-Identifier: GPL-3.0-or-later
#
# Phase 0 bootstrap entry point.
#
# The legacy project tree still lives in gr-uwb/ and is promoted to the
# repository root in Phase 3A. This forwarding project exists only so the
# repository-root CMake presets (build/<preset>) can configure and build it.
# It declares no targets and no policy of its own.
#
# TODO(openuwb-arch-phase3): Phase 3A promotes gr-uwb/CMakeLists.txt to this
# path and deletes this file together with the gr-uwb/ shell.
cmake_minimum_required(VERSION 3.8)
project(OpenUWB CXX C)
enable_testing()
add_subdirectory(gr-uwb)
