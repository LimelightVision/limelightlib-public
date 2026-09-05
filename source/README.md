# LimelightLib portable sources

These are the library files behind the LimelightLib vendordep, one folder per
WPILib train. Prefer the vendordep. Use these files only when you need to
modify the library or cannot use vendordeps.

| Folder | WPILib | NetworkTables timestamps |
|---|---|---|
| `alpha7/` | 2027 alpha-7 and newer | nanoseconds |
| `alpha5-6/` | 2027 alpha-5 and alpha-6 | microseconds |

## Java

Copy `Limelight.java` into your robot project as
`src/main/java/first/Limelight.java`. It declares `package first;`, so import it
with `import first.Limelight;`. Change the package line if you move it.

## C++

Copy `Limelight.h` into your robot project as `src/main/include/Limelight.h`
and `#include "Limelight.h"`. The library is header-only and lives in the
`limelight` namespace.

## Requirements

Limelight OS 2027.0 or later, or Systemcore OS with built-in vision. Java 25 or
C++23. Both files are MIT licensed; see `LICENSE` in this folder.
