I will update the project documentation to reflect the new build system and artifact location.

### Implementation Plan

1.  **Update `README.md`**:
    *   Locate the "Installation & Usage" or "Compile" section.
    *   Update the build command from `colcon build` to `./build.sh`.
    *   Update the source command from `source install/setup.bash` to `source cache/install/setup.bash`.
    *   Add a note explaining that build artifacts are stored in `cache/` to keep the workspace clean.

2.  **Verify other docs**: Check if `docs/dev_instruction/` contains any "how to build" guides and update them if necessary.

I will start by reading `README.md`.
