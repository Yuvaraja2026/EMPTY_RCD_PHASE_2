# T0057 Radar Firmware

## Hardware

AWRL1432 / xWRL1432-class radar
Cortex-M4F

## Required Software

- Code Composer Studio: company-approved version
- MMWAVE-L-SDK: 05.05.04.02
- SysConfig: 1.28.0
- TI ARM Clang: 5.1.1.LTS

Do not build this project using another SDK version without
requalification.

## CCS Setup

1. Install CCS.
2. Install MMWAVE-L-SDK 05.05.04.02.
3. Start CCS.
4. Verify MMWAVE-L-SDK is detected by CCS.
5. Verify SysConfig 1.28.0 is detected.
6. Verify TI ARM Clang 5.1.1.LTS is detected.
7. Clone this repository.
8. Import the project into CCS.
9. Select Release configuration.
10. Clean Project.
11. Build Project.
12. Confirm .out and .appimage are generated.

## Target

AWRL1432
XDS110 USB Debug Probe

## Important

Do not modify the frozen golden baseline without creating
a new version/tag.