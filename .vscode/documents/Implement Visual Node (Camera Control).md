I will implement the Visual Node tasks as outlined in the `dev_diary`.

### 1. Vision SDK Setup

* **Action**: Verify the existence and compilation status of `OrbbecSDK_ROS2` in `src/support/sdk` 

* **Note**: The user has already marked "Finding Vision SDK" as complete, so I will focus on "Familiarizing" and "Publishing Node".

### 2. Implement `camera_control` Package

* **Action**: Create a new package `camera_control` in `src/execution/` to wrap the Orbbec SDK.

* **Details**:

  * Create `src/execution/camera_control/launch/camera.launch.py` which includes/wraps the standard Orbbec launch files.

  * Ensure topics are remapped correctly (e.g., to `/image_raw` as per requirements).

### 3. Verification & Testing

* **Action**: Build the package and perform a dry-run launch test (to check for parameter errors).

* **Rule Compliance**: All build outputs will be redirected to `cache/` as per the new rule.

### 4. Documentation

* **Action**: Update `docs/dev_diary/2025.1.24.md` to mark tasks as complete after verification.

