all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build_gpu          - build gpu images"
	@echo "   1. make build_cpu          - build cpu images"
	# @echo "   1. make pull             - pull all images"
	# @echo "   1. make clean            - remove all images"
	@echo ""

build_gpu:
		@docker build --tag=ruffsl/ros_caffe:gpu gpu/.

build_cpu:
		@docker build --tag=ruffsl/ros_caffe:cpu cpu/.

# pull:
# 	@docker pull ros

# clean:
# 	@docker rmi -f ros
