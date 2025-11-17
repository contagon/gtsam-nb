#!/bin/bash
if [ ! -d ".vcpkg/" ]; then
    git clone https://github.com/microsoft/vcpkg.git .vcpkg/
fi
cd .vcpkg
git switch --detach 2025.10.17
cd ..
./.vcpkg/bootstrap-vcpkg.sh