Import("env")
import sys, os, shutil

# ── Патч флагов FPU ─────────────────────────────────────────
REMOVE = {"-mfloat-abi=softfp", "-mfloat-abi=soft", "-mfpu=vfpv3-d16"}
ADD    = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

for k in ("CCFLAGS", "CXXFLAGS", "ASFLAGS", "LINKFLAGS"):
    env[k] = [f for f in env.get(k, []) if f not in REMOVE]
    for flag in ADD:
        if flag not in env[k]:
            env[k].append(flag)

# ── Кроссплатформенная загрузка через STM32CubeProgrammer ────
_cli_env = os.environ.get("STM32_PROGRAMMER_PATH", "")

if _cli_env:
    _cli = os.path.join(_cli_env, "STM32_Programmer_CLI")
elif sys.platform.startswith("win"):
    _win_default = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    _cli = _win_default if os.path.isfile(_win_default) else "STM32_Programmer_CLI"
else:
    _cli = shutil.which("STM32_Programmer_CLI") or "STM32_Programmer_CLI"

env.Replace(
    UPLOADER=_cli,
    UPLOADCMD=f'"{_cli}" -c port=SWD mode=UR -w "$SOURCE" -v -rst'
)
