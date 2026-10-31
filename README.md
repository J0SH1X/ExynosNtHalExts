# NT HAL extensions
HAL extensions for ARMv7 NT on the Samsung Galaxy Note (GT-N7000, Exynos4210). Forked from [HTC-Leo-Revival-Project/NtHalExts](https://github.com/HTC-Leo-Revival-Project/NtHalExts); headers and libraries from the (Win8 M2-era) NT DDK are reproduced here for ease of building.

You need correct `CSRT` ACPI tables to load these (Exynos441xPkg provides them).

## halextmct
HAL extension for the Samsung Exynos Multi-Core Timer (MCT), for SoCs where the HAL has no usable ARM generic timer (Exynos4's Cortex-A9 has none; Exynos5 firmware leaves CNTFRQ unprogrammed):

- counter: the free-running 64-bit global counter (`G_CNT`)
- interrupt timer: global comparator 0 (`G_COMP0`), one-shot and pseudo-periodic

Only one processor is covered, like Linux's `mct-comp` clock event on CPU0.

One DLL for every Exynos: the firmware describes the MCT in the CSRT resource group (VendorId `SAMS`, DeviceId = SoC, e.g. `0x5420` → `OSExtensionDatabase\VEN_SAMS&DEV_5420`):

- a timer resource descriptor (Type 2, Subtype 0): MCT base, size, counter frequency, `G_COMP0` GSI
- optionally, in the group's shared info: version 1, base and size of an uncached OS log ring for breadcrumbs (`usbdbg.py oslog`)

Exynos5420Pkg (v1awifi) provides such a CSRT: MCT at `0x101C0000`, 24 MHz, `G_COMP0` via interrupt combiner 23.3 = GSI 55.  Exynos441xPkg still has the old descriptor-less group and needs the timer descriptor added (MCT at `0x10050000`, GSI 89).

### Building (Linux, no Visual Studio)
Needs clang, lld-link and osslsigncode, plus the Windows SDK and MSVC CRT headers, e.g. from [xwin](https://github.com/Jake-Shadle/xwin):

    xwin --accept-license --arch x86_64 splat --output ~/s2/winsdk/splat
    halextmct/build.sh          # WINSDK=<splat dir> to override

Only headers are used (the current SDK has no ARM32 libraries, and none are needed). `build.sh` test-signs the DLL with `~/s2/winsdk/testsign/testsign.{crt,key}` (`SIGN=` to override); the certificate needs the Code Signing and HAL Extension (`1.3.6.1.4.1.311.61.5.1`) EKUs.

`-DLOAD_TEST=1` makes the extension bugcheck with `0x4D435421` ("MCT!", parameters VendorId, DeviceId, handle) as soon as the HAL calls it, to check that it is found and loaded at all.

### Installing
Into a mounted image (e.g. `boot.wim`):

    dism /Image:<mount> /Add-Driver /Driver:halextmct.inf /ForceUnsigned

and `bcdedit /set {default} testsigning on` in the BCD that boots it. `halextmct.reg` does the same registry change by hand (e.g. `hivexregedit --merge`).
