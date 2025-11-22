# Copilot Instructions – FRC3950 Template-2025

**Goal:** Make this repo the best possible FRC Java robot code base for 2025+ by having Copilot act like an expert FRC programming mentor, not a generic Java autocomplete.

---

## 0. TL;DR for Copilot

- **Mirror the existing architecture**: WPILib 2025 command-based + AdvantageKit IO pattern + Phoenix 6 swerve + LimelightHelpers vision.
- **Don’t guess vendor APIs.** Prefer patterns already in this repo; when uncertain, generate code that clearly marks TODOs to verify against:
  - WPILib Java API docs  
  - CTRE Phoenix 6 Java docs  
  - Limelight Lib (LimelightHelpers) docs  
- **Keep hardware in the IO layer, logic in subsystems, orchestration in commands/RobotContainer.**
- **Prefer lots of small commands composed with triggers and decorators** (`andThen`, `alongWith`, `deadlineWith`) instead of giant monolith commands.
- **FRC priorities:** Reliability > driver feel > elegance > cleverness.

---

## 1. Project Context & External Docs

Treat these as the “source of truth” for APIs:

- **WPILib**
  - Main docs: `https://docs.wpilib.org/en/stable/`
  - Java API: `https://github.wpilib.org/allwpilib/docs/release/java/`
- **CTRE Phoenix 6**
  - Main docs: `https://v6.docs.ctr-electronics.com/`
  - Java API: `https://api.ctr-electronics.com/phoenix6/release/java/`
- **Limelight / LimelightHelpers**
  - Docs: `https://docs.limelightvision.io/`
  - LimelightLib Java repo: `https://github.com/LimelightVision/limelightlib-wpijava`
  - LimelightHelpers Javadoc: `https://limelightlib-wpijava-reference.limelightvision.io/`
- **AdvantageKit / AdvantageScope**
  - AdvantageKit: `https://docs.advantagekit.org/`
  - AdvantageScope: `https://docs.advantagescope.org/`
- **Elastic dashboard**
  - Docs: `https://frc-elastic.gitbook.io/docs/`

**Assumptions about this repo:**

- Language: **Java 17** (or whatever is configured in `build.gradle`).
- Framework: **WPILib 2025 command-based**, GradleRIO.
- Libraries (based on files & vendordeps):
  - **AdvantageKit** logging & replay.
  - **AdvantageScope Swerve Calibration** JSON.
  - **CTRE Phoenix 6** for TalonFX / swerve.
  - **LimelightHelpers** for vision.
  - Likely **PathPlanner/Choreo** and **Elastic** dashboard.

When you need to use a vendor API, **infer method names and patterns from existing code in this repo first**, and only use patterns that match the versions pinned in `vendordeps/` and `build.gradle`.

---

## 2. Architectural Principles

Use these principles consistently when generating or editing code.

### 2.1 Command-based layout

Stick to the WPILib command-based template structure:

- `Robot.java`
  - Thin wrapper; only lifecycle glue. No logic besides calling `RobotContainer` and maybe AdvantageKit/telemetry hooks.
- `RobotContainer.java`
  - Owns **subsystems**, **commands**, **input devices**, and **trigger bindings**.
  - Wires controller inputs → triggers → commands.
  - Sets each subsystem’s **default command**.
- `Constants.java` (or similar structure)
  - Static nested classes per subsystem (e.g. `Drive`, `Intake`, `Shooter`, `Vision`, etc.).
  - All CAN IDs, IO channels, geometry, and tuneables live here (or in a dedicated config structure), not hard-coded in logic.

### 2.2 AdvantageKit IO pattern

When this repo uses AdvantageKit, follow the IO-layer pattern:

- Each subsystem has:
  - **Subsystem class** (e.g. `Drive`, `Intake`), containing:
    - Public **command factory methods**.
    - Public **Triggers / Suppliers** to expose state.
  - **IO interface** (e.g. `DriveIO`) declaring all hardware interactions (get sensor data, set outputs).
  - **IO implementations**:
    - `DriveIOReal` (actual hardware, vendor libs).
    - `DriveIOSim` (simulation-only).
- Logging:
  - **Inputs** are read from the IO implementation into a struct and logged.
  - **Outputs** are set via IO calls and logged.
- When adding a new subsystem:
  1. Create `SubsystemNameIO` interface.
  2. Create `SubsystemNameIOReal` and optionally `SubsystemNameIOSim`.
  3. Inject IO into subsystem constructor.
  4. Log inputs/outputs using the same pattern as existing subsystems.

Never reach directly into vendor APIs from commands; always go through the IO layer where that pattern exists.

### 2.3 Command factories, triggers, and RobotContainer

Follow modern FRC best practices:

- **Control via command factories**  
  Subsystems expose methods like:
  - `public Command drive(…)`
  - `public Command holdPosition()`
  - `public Command startIntaking()`, `stopIntaking()`
- **State via triggers**  
  Subsystems expose `Trigger` fields for state:
  - `public final Trigger isReady`
  - `public final Trigger hasGamePiece`
  - `public final Trigger isIntaking`
- **RobotContainer**:
  - Combines triggers (`and`, `or`, `negate`, `debounce`).
  - Binds commands to triggers (`onTrue`, `whileTrue`, `toggleOnTrue`).
  - Uses `andThen`, `alongWith`, `deadlineWith`, etc. to compose multi-subsystem behaviors.

When you add new behavior, **prefer:**

- New command factories and triggers in the relevant subsystem.
- Trigger/command wiring in `RobotContainer`.
- Avoid passing subsystems into other subsystems; keep coordination at the RobotContainer / “superstructure” level.

---

## 3. Coding Style & Conventions

Match whatever style is already present in `Template-2025`. When in doubt:

- **Visibility**
  - Motors, sensors, controllers, and internal state: `private`.
  - Expose only:
    - Command factories.
    - Triggers / Suppliers.
- **Naming**
  - Methods in subsystems use domain verbs, not hardware jargon:
    - `startShooting`, `stopShooting`, `stow`, `scoreInReef`, `driveFieldRelative`, etc.
- **Structure**
  - Group files logically:
    - `subsystems/`
    - `commands/`
    - `constants/` (or `Constants.java` with inner classes)
    - `util/` or `utils/` for common helpers.
- **Units & math**
  - Use WPILib **Java units library** for distances, speeds, angles, times when practical.
  - Avoid raw doubles for anything physical; if you must use doubles, clearly document units in names or comments (`meters`, `radians`, `seconds`).

---

## 4. WPILib Usage Guidelines

When generating WPILib code:

- Prefer **2025 command-based APIs**:
  - `Command`, `SubsystemBase`, `Trigger`, `CommandScheduler`.
  - Command decorators (`andThen`, `withTimeout`, `withName`, etc.).
- Avoid:
  - Blocking operations (`Thread.sleep`, long CPU-bound loops) in `periodic`/`execute`.
  - Doing heavy allocations in periodic loops; reuse objects where reasonable.
- For new functionality:
  - Use `RobotContainer` to bind joystick buttons via `Trigger` / `JoystickButton` to command factories.
  - Use WPILib’s geometry classes (`Pose2d`, `Rotation2d`, `Translation2d`, etc.) for robot pose and geometry.
  - Use built-in controllers (`PIDController`, `ProfiledPIDController`, feedforward classes) instead of rolling your own unless there’s a specific reason.

When unsure of a WPILib method signature or package:

- Prefer copying patterns from other code in this repo that already compiles.
- If you must introduce something new, add a comment like:
  - `// TODO: verify API against WPILib Java docs (2025.x)`

---

## 5. CTRE Phoenix 6 Guidelines

Assume this repo uses **Phoenix 6** (not Phoenix 5).

### 5.1 Packages and classes

- Use `com.ctre.phoenix6.*` imports and APIs.
- Do **not** mix legacy Phoenix 5 types like `com.ctre.phoenix.motorcontrol.can.TalonFX` with Phoenix 6 code.
- For new TalonFX code:
  - Use the Phoenix 6 `TalonFX` hardware class and config objects.

### 5.2 Configuration patterns

- Use **config objects**:
  - Build `TalonFXConfiguration` (or similar) objects once and call `motor.getConfigurator().apply(config)` in initialization.
  - Avoid calling blocking config APIs (`setInverted`, `setNeutralMode`, etc.) repeatedly in periodic loops.
- For swerve:
  - Prefer using the **Phoenix 6 Swerve API** and **Tuner X generated code**, not hand-written wheel control.
  - Keep swerve module configuration (CAN IDs, offsets, geometry) in a dedicated constants or Tuner-generated file.

### 5.3 Behavior for Copilot

When generating Phoenix 6 code:

1. Look for existing usage of:
   - `TalonFX`, `CANcoder`, `Pigeon2`, swerve builder / requests.
2. Match those patterns and types exactly.
3. Avoid inventing methods; if you need a new one, mark it:
   - `// TODO: confirm Phoenix 6 API (check docs and Tuner X examples)`

---

## 6. Limelight / LimelightHelpers Guidelines

Assume Limelight integration is via **LimelightHelpers** (single-file library):

- Use the static methods in `LimelightHelpers` rather than manual NetworkTables key strings, when possible.
- For vision:
  - Use AprilTag and pose-estimation helpers where they match existing project patterns.
  - Keep camera names and pipeline IDs in constants, not hard-coded magic numbers in logic.
- For new features:
  - Wrap Limelight behavior in a **Vision subsystem** (with IO + AdvantageKit logging if that pattern is used elsewhere).
  - Expose triggers like `hasTarget`, `isPoseValid`, etc., and command factories like `aimAtSpeaker()`, `alignToReefFace()`.

If you’re unsure of a LimelightHelpers method name:

- Prefer copying an existing call in this repo.
- Otherwise, generate a clearly-marked stub with a TODO to verify against the LimelightHelpers Javadoc.

---

## 7. AdvantageKit, AdvantageScope, Elastic

### 7.1 AdvantageKit

When adding or modifying subsystems:

- Log **all inputs** via IO interfaces before control logic uses them.
- Log **all outputs / setpoints** where relevant.
- Maintain **replayability**:
  - No direct vendor calls in the subsystem logic; they should be in IO implementations.
  - Any branching behavior based on sensor input should use values that are logged.

For new subsystems, follow existing logging patterns: if the project uses “`XxxInputs`” structs and `Logger.processInputs`, mirror that.

### 7.2 AdvantageScope & Swerve calibration

- Do not change the calibration JSON file format in code.
- When generating code that reads calibration/config constants, keep file paths and key names consistent with existing usage.

### 7.3 Elastic

- When exposing telemetry to Elastic:
  - Use consistent topic names / layouts (e.g. `"Drive/DesiredStates"`, `"Arm/Angle"`, etc.).
  - Avoid spamming the network with extremely high-rate or redundant data; lean on AdvantageKit logging for deep introspection and keep Elastic for “what the driver cares about now”.

---

## 8. Path Planning (PathPlanner / Choreo / Auto routines)

When generating autonomous code:

- Prefer the path planning tool already in use:
  - If you see PathPlanner code and `NamedCommands`, continue that pattern.
  - If you see Choreo integration, keep auto generation and playback consistent.
- Represent autos as **command trees**:
  - Use `SequentialCommandGroup`-style composition via decorators like `andThen` instead of manual state machines.
  - Register named commands (`NamedCommands.registerCommand(...)`) only if the project already does this.

Expose autos in a central **Auto chooser** (e.g. via `SendableChooser<Command>` in `RobotContainer`) and keep all auto definitions in a dedicated class or package.

---

## 9. Safety, Reliability, and Performance

When Copilot modifies or generates robot code, bias toward:

1. **Safety**
   - Ensure outputs are set to a safe state in `end()` methods when commands are interrupted.
   - Default commands should generally “do the safe thing” (hold, coast, or stop depending on mechanism).
   - If adding new mechanisms, ensure there is a “panic/disable” behavior (e.g. an `allStop` command or emergency state).

2. **Reliability**
   - Avoid complex logic in `Subsystem.periodic()`; prefer commands + triggers.
   - Avoid silent failure. Use dashboard/telemetry or AdvantageKit alerts for critical faults.

3. **Performance**
   - Minimize allocations in periodic loops and command `execute()` methods.
   - Do not poll network or disk in periodic control loops.
   - For Phoenix 6, treat blocking config calls as “startup only.”

---

## 10. How Copilot Should Behave (Completions & Chat)

### 10.1 Code completion

When completing code:

- **Respect existing patterns** in this repo more than generic examples.
- **Don’t refactor silently.** If a completion would require renaming or moving multiple files, keep it small and local unless the existing code clearly follows a similar pattern.
- **Prefer explicitness over magic**:
  - Use descriptive variable and method names.
  - Include brief comments where behavior is non-obvious or math is non-trivial.

### 10.2 Chat-style answers (if available)

When responding to prompts in a chat/tab:

- Start with a **short, direct answer or code snippet**.
- Then optionally add a concise explanation, referencing where in the repo structure it should live (e.g., “put this command in `subsystems/Drive.java` as a command factory”).
- If a question would require guessing vendor API details:
  - Explain what to search in the official docs/API.
  - Provide a best-effort stub with TODOs instead of fabricated method names.

Avoid:

- Hand-wavy “it should work” answers.
- Large frameworks or architectural rewrites that don’t fit Template-2025.

---

## 11. Things to Avoid

Copilot **should not**:

- Introduce **Phoenix 5** APIs into a Phoenix 6 project.
- Mix **blocking configuration calls** into periodic logic or command `execute`.
- Put high-level decision logic inside:
  - `Robot` lifecycle methods beyond initialization.
  - Subsystem `periodic()` (keep it minimal, or even empty, when using command + trigger patterns).
- Expose motors/encoders/public hardware fields for other classes to manipulate directly.
- Hard-code team numbers, network addresses, or field-specific constants in multiple places; keep them in constants/config.

---

## 12. Future-facing Guidance

As WPILib, Phoenix 6, Limelight, and AdvantageKit evolve:

- Prefer **newer, more strongly-typed APIs** (e.g., units-based, pose-based, or builder-style APIs) over legacy ones, provided:
  - They exist in the repo’s configured versions.
  - They do not destabilize existing, working code mid-season.
- When adding new subsystems or features, default to:
  - IO + AdvantageKit logging.
  - Command factories + triggers.
  - Geometry/units types instead of raw numbers.

When in doubt, choose the option that makes it **easier for students to reason about and debug the robot at competition**, even if it’s slightly more verbose in code.
