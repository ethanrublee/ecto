# 
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${ecto_SOURCE_DIR}/src/lib)

ectomodule(ecto
  cell.cpp
  ecto.cpp
  except.cpp
  plasm.cpp
  parameters.cpp
  registry.cpp
  schedulers.cpp
  strand.cpp
  streambuf.cpp
  tendril.cpp
  tendrils.cpp
  cells/And.cpp
  cells/BlackBox.cpp
  cells/Constant.cpp
  cells/Dealer.cpp
  cells/Counter.cpp
  cells/ether.cpp
  cells/If.cpp
  cells/Passthrough.cpp
  cells/PassthroughN.cpp
  cells/TrueEveryN.cpp
  )

set_target_properties(ecto_ectomodule PROPERTIES
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/share/ecto/python"
  )

#this is the python extension
install(TARGETS ecto_ectomodule
  DESTINATION ${ecto_PYTHON_INSTALL}
  COMPONENT main
  )


