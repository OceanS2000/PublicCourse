simple_simulation:
	bazel build -c opt //pnc:simulation_main
	PONY_ROOT=/home/ocean/PublicCourse SVGA_VGPU10=0 LC_ALL=C \
	./bazel-bin/pnc/simulation_main --multi_process_mode=false

replay:
	bazel build -c opt //pnc:replay_main
	PONY_ROOT=/home/ocean/PublicCourse SVGA_VGPU10=0 LC_ALL=C \
	./bazel-bin/pnc/replay_main --simulation_log_file_path=/tmp/simulation_log.bin

clean:
	rm /tmp/simulation_log*
