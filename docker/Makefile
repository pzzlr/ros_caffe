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
		@docker build --tag=gpu:indigo-caffe gpu/caffe/.
		@docker build --tag=gpu:indigo-ros-caffe gpu/ros-caffe/.
		# @docker build --tag=gpu:indigo-ros-caffe-dev gpu/dev/.

build_cpu:
		@docker build --tag=cpu:indigo-caffe cpu/caffe/.
		@docker build --tag=cpu:indigo-ros-caffe cpu/ros-caffe/.
		# @docker build --tag=cpu:indigo-ros-caffe-dev cpu/dev/.

# pull:
# 	@docker pull ros

# clean:
# 	@docker rmi -f ros
