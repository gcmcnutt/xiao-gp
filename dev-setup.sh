#!/bin/bash
# Development setup script for xiao-gp
# This script allows developers to use a local GP clone instead of the submodule

GP_LOCAL_PATH="${1:-../GP}"

echo "Setting up development environment..."

if [ -L include/GP ]; then
    echo "GP is already symlinked to: $(readlink include/GP)"
    exit 0
fi

if [ -d include/GP/.git ]; then
    echo "Converting GP submodule to symlink..."
    
    # Deinitialize and remove submodule
    git submodule deinit -f include/GP
    git rm --cached include/GP
    rm -rf include/GP
    
    # Create symlink
    ln -s "$(realpath "$GP_LOCAL_PATH")" include/GP
    
    echo "GP submodule replaced with symlink to: $(realpath "$GP_LOCAL_PATH")"
    echo "Note: This change is local only and should not be committed to git"
else
    echo "GP directory not found or not a git repository"
    exit 1
fi

echo "Development setup complete!"
echo ""
echo "To restore submodule mode:"
echo "  rm include/GP"
echo "  git submodule update --init include/GP"