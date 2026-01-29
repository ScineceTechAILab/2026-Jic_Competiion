I will move the existing `install`, `log`, and `build` directories to a new `cache` directory and configure `colcon` to use this new location for future builds.

### Implementation Plan

1. **Create Cache Directory**: Create the `cache` directory in the project root.

2. **Move Existing Artifacts**: Move `build`, `install`, and `log` folders into `cache/`.

3. **Update Build Workflow**:

   * Since `colcon` defaults to the current directory, I cannot permanently change its default output directories via a config file in the project root (colcon configuration is typically user-level).

   * **Action**: I will create a convenience script `build.sh` (or `Makefile`) that wraps the `colcon build` command with the custom output paths:

     * `--build-base cache/build`

     * `--install-base cache/install`

     * `--log-base cache/log`

   * This ensures consistency for future builds.

4. **Update** **`.gitignore`**: Ensure `cache/` is ignored by git.

I will proceed with these changes.
