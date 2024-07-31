# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sakura/DoubleLens/build/_deps/embree_binary-src"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-build"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/tmp"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/src/embree_binary-populate-stamp"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/src"
  "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/src/embree_binary-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/src/embree_binary-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sakura/DoubleLens/build/_deps/embree_binary-subbuild/embree_binary-populate-prefix/src/embree_binary-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
