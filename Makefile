simple_simulation:
	bazel build -c opt //pnc:simulation_main
	PONY_ROOT=/home/ocean/PublicCourse SVGA_VGPU10=0 LC_ALL=C \
	../bazel-bin/pnc/simulation_main --multi_process_mode=false