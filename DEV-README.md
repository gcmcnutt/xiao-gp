# Development Setup

## GP Submodule vs Local Development

This project uses the GP (Genetic Programming) library as a git submodule by default. However, for active development on both the xiao-gp project and the GP library, you may want to use a local clone of GP instead.

### Default Mode (Submodule)
```bash
# Initialize submodules (for new clones)
git submodule update --init --recursive

# Update submodules to latest
git submodule update --remote
```

### Development Mode (Local Symlink)
```bash
# Switch to using local GP clone (assumes ../GP exists)
./dev-setup.sh

# Or specify custom path
./dev-setup.sh /path/to/your/GP/clone
```

### Switching Back to Submodule Mode
```bash
# Remove symlink and restore submodule
rm include/GP
git submodule update --init include/GP
```

## Benefits of Each Approach

**Submodule Mode:**
- ✅ Consistent versions for all developers
- ✅ Automatic tracking of specific GP commits
- ✅ Safe for production/release builds

**Development Mode (Symlink):**
- ✅ Live changes to GP library visible immediately
- ✅ Can work on GP and xiao-gp simultaneously  
- ✅ No need to commit/push GP changes to test
- ✅ Easier debugging across both codebases

## Current Status

You can check which mode you're in:
```bash
ls -la include/GP
```

- If it shows a symlink (`->`) you're in development mode
- If it shows a directory, you're in submodule mode