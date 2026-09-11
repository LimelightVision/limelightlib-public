# LimelightLib portable sources

These are the library files behind the LimelightLib vendordep, one folder per
WPILib train. Prefer the vendordep. Use these files only when you need to
modify the library or cannot use vendordeps.

| Folder | WPILib | NetworkTables timestamps |
|---|---|---|
| `alpha7/` | 2027 alpha-7 and newer | nanoseconds |
| `alpha5-6/` | 2027 alpha-5 and alpha-6 | microseconds |

## Java

Copy the `java/` folder into your robot project as
`src/main/java/com/limelightvision/`. The files declare
`package com.limelightvision;`, one public type per file, so import them with
`import com.limelightvision.Limelight;` and so on. Change the package line in
every file if you move them.

## C++

Copy the contents of the `cpp/` folder into your robot project's
`src/main/include/` so you have `src/main/include/Limelight.h` and
`src/main/include/limelight/`. Then `#include "limelight/Limelight.h"` (the
top-level `Limelight.h` is a one-line shim that does the same). The library is
header-only, one type per header, in the `limelight` namespace.

## Requirements

Limelight OS 2027.0 or later, or Systemcore OS with built-in vision. Java 25 or
C++23. All files are MIT licensed; see `LICENSE` in this folder.
