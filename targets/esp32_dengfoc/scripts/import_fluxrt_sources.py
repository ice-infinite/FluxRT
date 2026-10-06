from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR")).resolve()
repo_root = project_dir.parents[1]
foc_include = repo_root / "foc" / "include"
platform_dir = repo_root / "foc" / "platform" / "esp32_dengfoc"

env.Append(CPPPATH=[str(foc_include), str(platform_dir)])
env.BuildSources(
    "$BUILD_DIR/fluxrt_foc_runtime",
    str(repo_root / "foc" / "runtime"),
    src_filter=[
        "+<foc_feedback_adapter.c>",
        "+<foc_as5600.c>",
        "+<foc_time_sync.c>",
        "+<foc_time_sync_wire.c>",
    ],
)
env.BuildSources(
    "$BUILD_DIR/fluxrt_dengfoc_platform",
    str(platform_dir),
    src_filter=[
        "+<foc_board_dengfoc_v04.c>",
        "+<foc_dengfoc_as5600_port.c>",
        "+<foc_dengfoc_current_sense.c>",
        "+<foc_dengfoc_power_stage.c>",
        "+<foc_time_sync_dengfoc.c>",
    ],
)
