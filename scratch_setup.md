# T0057 Radar Firmware — Scratch Setup for a New Intern

> **Goal:** A new intern should be able to go from a fresh Windows PC to:
>
> **Install tools → clone Git repo → import into CCS → Clean → Build successfully**
>
> No manual include-path editing should be required.

---

## 0. Project build versions — do not change

This project is currently configured for:

| Item | Required version / target |
|---|---|
| Radar device | **AWRL1432 / xWRL14xx class** |
| CPU | **Cortex-M4F** |
| MMWAVE-L-SDK | **05.05.04.02** |
| SysConfig | **1.28.0** |
| TI ARM Clang | **5.1.1.LTS** |
| Recommended build | **Release** |
| Debug probe | **XDS110** |

These values come from the checked-in CCS project metadata.

**Do not upgrade the SDK, SysConfig, compiler, or device configuration during initial setup.**

If another version is required later, it must be treated as a separate controlled upgrade and revalidated.

---

# 1. What the intern needs before starting

A Windows PC with:

- Internet access for the first tool installation
- Git
- Code Composer Studio (CCS), using the team-approved Theia-based CCS installer
- MMWAVE-L-SDK **05.05.04.02**
- TI ARM Clang **5.1.1.LTS**
- SysConfig **1.28.0**
- Access to the company Git repository

Hardware is not required just to compile, but for board testing you also need:

- AWRL1432/xWRL1432 radar board
- USB cable
- XDS110 connection available on the board

---

# 2. Install Git

Download and install Git for Windows using the company-approved installer.

During installation, default options are normally sufficient.

After installation:

1. Open **Command Prompt**.
2. Run:

```bat
git --version
```

Expected result:

```text
git version x.x.x
```

If Windows says `git is not recognized`, close/reopen Command Prompt or reinstall Git.

---

# 3. Install Code Composer Studio

Install the **same team-approved CCS version used for the validated project**.

Use the normal TI installation location unless company IT requires something else.

Example:

```text
C:\ti\
```

After installation:

1. Launch CCS once.
2. Allow CCS to finish first-start initialization.
3. Close CCS.

> **Important:** The project metadata confirms it is a **Theia-based CCS project**, but the source project alone does not reliably identify the human-facing CCS installer version.  
> The project lead should record the approved CCS installer/version in `DEPENDENCIES.md`.

---

# 4. Install MMWAVE-L-SDK 05.05.04.02

Install exactly:

```text
MMWAVE-L-SDK 05.05.04.02
```

For the easiest setup, install it in the default TI directory:

```text
C:\ti\MMWAVE_L_SDK_05_05_04_02
```

After installation, verify that the folder exists.

A typical installation should contain directories such as:

```text
C:\ti\MMWAVE_L_SDK_05_05_04_02\
    examples\
    firmware\
    source\
    docs\
```

Do **not** copy individual SDK header files into the T0057 project.

The CCS project already uses SDK variables such as:

```text
${MCU_PLUS_SDK_PATH}
${COM_TI_MMWAVE_L_SDK_INCLUDE_PATH}
${CG_TOOL_ROOT}
```

CCS must resolve these from the installed products.

---

# 5. Confirm required TI tools in CCS

Open CCS.

Verify CCS can see:

```text
MMWAVE-L-SDK 5.5.4.02
SysConfig 1.28.0
TI ARM Clang 5.1.1.LTS
```

Depending on the CCS version, these may appear under product/compiler settings or when the project is imported.

If CCS reports that a product/compiler is missing:

**STOP.**

Do not manually add random include paths.

Install or register the exact missing product/version first.

---

# 6. Create a clean working folder

Do not clone the project inside an old CCS workspace.

Recommended:

```text
C:\work\
```

Example:

```text
C:\work\Radar\
```

Avoid:

```text
Desktop\old_project_copy\
Downloads\project_final_final2\
another engineer's CCS workspace\
```

Each developer should have their own clean Git clone.

---

# 7. Clone the Git repository

Open Command Prompt.

Go to the working directory:

```bat
cd /d C:\work
```

Clone:

```bat
git clone <REPOSITORY_URL>
```

Example:

```bat
git clone https://git.company.com/radar/T0057-Radar.git
```

Enter the repo:

```bat
cd Radar
```

Check status:

```bat
git status
```

Expected:

```text
On branch ...
nothing to commit, working tree clean
```

---

# 8. Expected repository structure

The repository should look like this:

```text
T0057-Radar/
│
├── firmware/
│   ├── .project
│   ├── .cproject
│   ├── .ccsproject
│   ├── .settings/
│   ├── targetConfigs/
│   │
│   ├── example.syscfg
│   ├── linker.cmd
│   ├── makefile_ccs_bootimage_gen
│   │
│   ├── main.c
│   ├── hwa.c
│   ├── hwa.h
│   ├── cfar.c
│   ├── doa.c
│   ├── tracker.c
│   ├── rangeproc.c
│   ├── radar.c
│   ├── radar_memory.h
│   ├── interrupts.c
│   ├── mmw_func.c
│   ├── mmw_flash_cal.c
│   ├── mmwave_control_config.c
│   └── ...
│
├── tools/
│   └── radar_logger.py
│
├── validation/
│   └── ...
│
├── docs/
│   └── ...
│
├── scratch_setup.md
├── DEPENDENCIES.md
├── CHANGELOG.md
└── .gitignore
```

The following folders should **not** be required from Git:

```text
Debug/
Release/
```

CCS generates them locally.

---

# 9. Import the project into CCS

Open CCS.

Use the project import command.

Select:

```text
C:\work\T0057-Radar\firmware
```

The folder to import is the folder containing:

```text
.project
.cproject
.ccsproject
```

Import the existing CCS project.

After import, the project name may still appear as:

```text
empty_xwrL14xx-evm_m4fss0-0_freertos_ti-arm-clang
```

That is currently expected.

Do not rename it during first-time setup.

---

# 10. Wait for CCS dependency resolution

After import, CCS may need a short time to:

- discover the MMWAVE-L-SDK
- discover SysConfig
- discover the compiler
- index source code
- generate SysConfig-derived files

Do not immediately add manual include paths if the editor temporarily shows unresolved includes.

First allow product discovery/indexing to finish.

---

# 11. Confirm the active target

The checked-in project contains:

```text
targetConfigs/AWRL1432.ccxml
```

and is configured for:

```text
AWRL1432
XDS110
Cortex-M4F
```

For build-only work, no board connection is required.

For debugging/running, confirm the AWRL1432 target configuration is active.

---

# 12. Select the Release build

For normal team validation, use:

```text
Release
```

In CCS:

1. Right-click the project.
2. Select **Build Configurations**.
3. Select **Set Active**.
4. Select **Release**.

Do not assume Debug and Release behave identically.

---

# 13. Clean the project

Before the first build:

```text
Project → Clean
```

or right-click the project and choose the clean command.

This removes old locally generated objects.

A fresh Git clone should not depend on another developer's build output.

---

# 14. Build the project

Build:

```text
Project → Build
```

or use the CCS build button.

The build should perform:

```text
SysConfig generation
→ C compilation
→ TI SDK library linking
→ linker
→ post-build boot-image generation
```

The post-build command uses the checked-in:

```text
makefile_ccs_bootimage_gen
```

and obtains SDK/compiler locations from CCS variables.

---

# 15. Build PASS criteria

The build is considered successful only when:

```text
**** Build Finished ****
```

appears with no errors.

The local `Release/` directory should contain build outputs including:

```text
<project-name>.out
empty.Release.map
<project-name>_linkInfo.xml
*.appimage
```

Exact app-image filenames may depend on the post-build configuration.

Do not commit the generated `Release/` directory.

---

# 16. If the build fails — do not start changing source code

Use this table first.

| Error | Most likely cause | Action |
|---|---|---|
| `MMWAVE-L-SDK product not found` | SDK missing/not registered | Install/register **5.5.4.02** |
| `SysConfig product not found` | Wrong/missing SysConfig | Install/register **1.28.0** |
| `TICLANG_5.1.1.LTS not found` | Compiler missing | Install exact compiler |
| TI header not found | SDK/product resolution problem | Do **not** manually copy headers |
| Library not found | Wrong SDK/version/path | Verify SDK **05.05.04.02** |
| `example.syscfg` errors | Wrong SysConfig/SDK/device | Verify pinned versions |
| Post-build make fails | CCS/SDK path/tool problem | Check CCS variables and SDK install |
| Old local absolute path appears | Generated Debug/Release reused | Delete local build dirs and clean build |

---

# 17. Important rule: never fix a new PC by adding random include paths

If a new intern sees:

```text
cannot open source file ...
```

they should **not** go to compiler settings and browse to arbitrary folders one by one.

The project is intentionally configured with portable SDK macros.

A missing header normally means:

```text
SDK missing
wrong SDK version
product not registered
compiler missing
SysConfig missing
```

Fix the dependency, not the project.

---

# 18. Quick 5-minute checklist after import

The intern should confirm:

- [ ] Git clone completed
- [ ] Working tree is clean
- [ ] `firmware/.project` exists
- [ ] `firmware/.cproject` exists
- [ ] `firmware/.ccsproject` exists
- [ ] `firmware/example.syscfg` exists
- [ ] MMWAVE-L-SDK **5.5.4.02** detected
- [ ] SysConfig **1.28.0** detected
- [ ] TI ARM Clang **5.1.1.LTS** detected
- [ ] Target is AWRL1432
- [ ] Active build = Release
- [ ] Clean completes
- [ ] Build completes with zero errors
- [ ] `.out` generated
- [ ] `.appimage` generated
- [ ] No manual include paths were added

---

# 19. Optional board run after successful build

After build success:

1. Connect the radar board over USB.
2. Confirm XDS110 is detected.
3. Open the AWRL1432 target configuration.
4. Start a debug session.
5. Load the newly built `.out`.
6. Reset if required.
7. Run.

For the current real-time firmware, UART output should eventually show tracker records similar to:

```text
T,<frame>,<numTargets>,<tid>,<x_mm>,<y_mm>,<vx_mmps>,<vy_mmps>
```

Example:

```text
T,101,1,3,125,2430,-20,-1120
```

Do not use an old `.out` from another computer.

---

# 20. Git rules for interns

Before starting work each day:

```bat
git pull
git status
```

Before editing:

```text
Make sure the working tree contains only the changes you intend to make.
```

After making a logical change:

```bat
git status
git diff
```

Commit with a meaningful message:

```bat
git add <files>
git commit -m "Describe the actual change"
git push
```

Do not commit:

```text
Debug/
Release/
*.out
*.obj
*.map
*.appimage
temporary logs
personal CCS workspace files
```

unless the team explicitly creates a release-artifact package outside normal source control.

---

# 21. When a new intern joins

The expected onboarding should be only:

```text
1. Install approved CCS
2. Install MMWAVE-L-SDK 05.05.04.02
3. Verify SysConfig 1.28.0
4. Verify TI ARM Clang 5.1.1.LTS
5. git clone
6. Import firmware/ into CCS
7. Select Release
8. Clean
9. Build
10. Done
```

There should be:

```text
NO copied include folders
NO copied Debug/Release folder
NO local D:\ path changes
NO manual linker-library browsing
NO source-code edits just to compile
```

If those are required, the repository setup is not yet portable and the problem should be fixed centrally.

---

# 22. One-time checklist for the project owner

Before telling interns this process is final, the project owner should test it once on a second clean PC.

Test exactly:

```text
Fresh PC
→ install pinned TI dependencies
→ git clone
→ import firmware/
→ Release
→ Clean
→ Build
```

PASS means:

```text
zero manual project-path edits
zero manual include additions
zero copied build folders
successful .out
successful .appimage
```

Only after that should this Git revision be tagged as the team onboarding baseline.

Suggested tag:

```text
T0057_PRE_TTC_AI_GOLDEN
```

---

# 23. If setup is broken, what should the intern send to the lead?

Do not send only "build failed."

Send:

1. Screenshot of the first build error.
2. Full CCS build console text.
3. Output of:

```bat
git status
git log -1 --oneline
```

4. Installed MMWAVE-L-SDK version.
5. Installed SysConfig version.
6. Installed TI ARM Clang version.
7. Whether build configuration is Debug or Release.
8. Whether they manually changed any CCS setting.

This is normally enough to reproduce the issue.

---

# Final expected onboarding experience

A correctly prepared repository should allow a new engineer to do this:

```text
Install approved TI tools
        ↓
git clone
        ↓
Import firmware/
        ↓
Select Release
        ↓
Clean
        ↓
Build
        ↓
SUCCESS
```

That is the standard for this project.
